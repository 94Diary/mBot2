"""Coordinate branch reservations and goal announcements for a small robot team.

The wire format is JSON in std_msgs/String. The ledger is deliberately free of
ROS imports so its decisions can be unit-tested without Gazebo or ROS running.
"""

import heapq
import json
import math
import time
import uuid
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


MATCH_RADIUS_M = 1.00
PEER_TIMEOUT_SECONDS = 4.0
DIRECTION_SECTORS = 4  # the current line maze has orthogonal branches


def direction_key(heading):
    """Turn a world-frame heading into a cardinal branch key."""
    return round((heading % (2 * math.pi)) * DIRECTION_SECTORS /
                 (2 * math.pi)) % DIRECTION_SECTORS


class TeamLedger:
    """Shared, observation-based junction/branch ledger; no preloaded maze."""

    def __init__(self, match_radius=MATCH_RADIUS_M):
        self.match_radius = match_radius
        self.junctions = {}
        self.next_junction_id = 1
        self.next_route_order = 1
        self.robots = {}
        self.goal = None
        self.decisions = {}
        # Shared map: where each robot left its last node, and links that
        # contradicted an earlier one (kept out of the map, reported).
        self.trails = {}
        self.link_conflicts = []
        # Head-on encounters: who hides where, and who waits for whom.
        self.encounters = {}
        self.next_encounter_id = 1
        # Corridors a robot gave up to a teammate it met head-on.
        self.avoid = {}

    def update_pose(self, report, now):
        robot = report['robot']
        previous = self.robots.get(robot)
        if previous is None:
            route_order = self.next_route_order
            self.next_route_order += 1
        else:
            route_order = previous['route_order']
        if report.get('route_order') is not None:
            route_order = int(report['route_order'])
        self.robots[robot] = {
            'x': float(report['x']),
            'y': float(report['y']),
            'heading': float(report.get('heading', 0.0)),
            'state': str(report.get('state', 'UNKNOWN')),
            'returning': bool(report.get('returning', False)),
            'yield_to': report.get('yield_to'),
            'yield_junction': report.get('yield_junction'),
            'yield_escape_heading': report.get('yield_escape_heading'),
            'blocked_behind': bool(report.get('blocked_behind', False)),
            'route_order': route_order,
            'seen_at': now,
        }

    def _junction(self, x, y, kind='JUNCTION'):
        if kind == 'START':
            # Robots leave the start box at different points: one START.
            start = next((n for n in self.junctions.values()
                          if n['kind'] == 'START'), None)
            if start is not None:
                return start
        else:
            for node in self.junctions.values():
                if (node['kind'] != 'START' and
                        math.hypot(node['x'] - x, node['y'] - y) <= self.match_radius):
                    if kind == 'JUNCTION':
                        node['kind'] = 'JUNCTION'  # a corner with a third arm
                    return node
        node_id = self.next_junction_id
        self.next_junction_id += 1
        node = {'id': node_id, 'kind': kind, 'x': x, 'y': y, 'branches': {}}
        self.junctions[node_id] = node
        return node

    @staticmethod
    def _branch(node, key, heading=None):
        branch = node['branches'].setdefault(
            key, {'status': 'OPEN', 'owner': None, 'visited_by': []})
        branch.setdefault('leads_to', None)
        if heading is not None and 'heading' not in branch:
            branch['heading'] = float(heading)
        return branch

    def _link(self, robot, node, approach_heading):
        """Join the branch this robot last left by to its way into node."""
        enter = direction_key(float(approach_heading) + math.pi)
        self._branch(node, enter, float(approach_heading) + math.pi)
        trail = self.trails.get(robot)
        if trail is None or trail[0] == node['id'] or trail[0] not in self.junctions:
            return
        previous, exit_dir = self.junctions[trail[0]], trail[1]
        out_branch = self._branch(previous, exit_dir)
        in_branch = node['branches'][enter]
        there = {'node': node['id'], 'enter_dir': enter}
        back = {'node': previous['id'], 'enter_dir': exit_dir}
        far = out_branch['leads_to']
        if (far not in (None, there) and in_branch['leads_to'] is None
                and self._split_edge(previous, exit_dir, node, enter, far)):
            return  # node sat unnoticed on a link recorded earlier
        if (out_branch['leads_to'] not in (None, there) or
                in_branch['leads_to'] not in (None, back)):
            # e.g. a robot backing through a node while yielding
            self.link_conflicts.append(
                f"{robot}: J{previous['id']}/{exit_dir} -> J{node['id']}/{enter} "
                f"contradicts {out_branch['leads_to']} / {in_branch['leads_to']}")
            return
        out_branch['leads_to'] = there
        in_branch['leads_to'] = back

    def _split_edge(self, previous, exit_dir, node, enter, far_link):
        """Insert node into the link previous/exit_dir -> far when it lies on it."""
        far = self.junctions.get(far_link['node'])
        if far is None or far['id'] == node['id']:
            return False
        detour = (math.hypot(node['x'] - previous['x'], node['y'] - previous['y']) +
                  math.hypot(far['x'] - node['x'], far['y'] - node['y']) -
                  math.hypot(far['x'] - previous['x'], far['y'] - previous['y']))
        if detour > 0.1:
            return False
        heading = math.atan2(far['y'] - node['y'], far['x'] - node['x'])
        onward = self._branch(node, direction_key(heading), heading)
        far_branch = far['branches'].get(far_link['enter_dir'])
        if onward['leads_to'] is not None or far_branch is None:
            return False
        previous['branches'][exit_dir]['leads_to'] = {'node': node['id'], 'enter_dir': enter}
        node['branches'][enter]['leads_to'] = {'node': previous['id'], 'enter_dir': exit_dir}
        onward['leads_to'] = dict(far_link)
        far_branch['leads_to'] = {'node': node['id'], 'enter_dir': direction_key(heading)}
        return True

    def report_node(self, report, now):
        """Record a start, corner or dead end a robot reached."""
        robot = report['robot']
        kind = report['kind']
        node = self._junction(float(report['x']), float(report['y']), kind)
        approach = float(report['approach_heading'])
        if kind == 'START':
            heading = float(report.get('heading', approach))
            self._branch(node, direction_key(heading), heading)
            self.trails[robot] = (node['id'], direction_key(heading))
            return node
        self._link(robot, node, approach)
        if kind == 'CORNER':
            exit_heading = float(report['exit_heading'])
            self._branch(node, direction_key(exit_heading), exit_heading)
            self.trails[robot] = (node['id'], direction_key(exit_heading))
        else:  # DEAD_END: the robot turns back the way it came
            self.trails[robot] = (node['id'], direction_key(approach + math.pi))
        return node

    def request_branch(self, report, now):
        robot = report['robot']
        request_id = report['request_id']
        token = (robot, request_id)
        if token in self.decisions:
            return self.decisions[token]

        node = self._junction(float(report['x']), float(report['y']))
        if report.get('approach_heading') is not None:
            self._link(robot, node, report['approach_heading'])
        # Arriving along a corridor is evidence that it has already been used.
        # Without this, a later approach from the opposite side can mistake
        # the original entrance for an unexplored straight-ahead branch.
        if report.get('approach_heading') is not None:
            inbound_key = direction_key(float(report['approach_heading']) + math.pi)
            inbound = node['branches'].setdefault(
                inbound_key, {'status': 'OPEN', 'owner': None, 'visited_by': []})
            arrived = inbound.setdefault('arrived_by', [])
            if robot not in arrived:
                arrived.append(robot)
            if robot not in inbound['visited_by']:
                inbound['visited_by'].append(robot)
        candidates = []
        branch_details = []
        if self.goal is not None:
            goal_node = next((n for n in self.junctions.values()
                              if n['kind'] == 'GOAL'), None)
            reach = (self._graph_distances([goal_node['id']], robot, now)
                     if goal_node is not None else {})
        else:
            frontier = [n['id'] for n in self.junctions.values()
                        if n['id'] != node['id'] and self._has_new_branch(n, robot, now)]
            reach = self._graph_distances(frontier, robot, now)
        for option in report['choices']:
            key = direction_key(float(option['heading']))
            branch = self._branch(node, key, option['heading'])
            branch_details.append({
                'name': option['name'], 'status': branch['status'],
                'visited_by': list(branch['visited_by']),
                'fallback': bool(option.get('fallback', option['name'] == 'back')),
                'failed_attempts': branch.get('failed_by', {}).get(robot, 0),
            })
            if option.get('fallback', option['name'] == 'back'):
                continue  # use the known inbound route only as a fallback
            if branch.get('hidden_by') not in (None, robot):
                continue  # a teammate is hiding in there
            heading = float(option['heading'])
            if branch['status'] == 'BLOCKED':
                continue
            # A robot that already explored this branch must try another one.
            # Teammates may still share it when every unclaimed path is busy.
            # Reservations and failed line capture do not prove a dead end.
            owner = branch['owner']
            owner_live = (owner in self.robots and
                          now - self.robots[owner]['seen_at'] < PEER_TIMEOUT_SECONDS)
            unclaimed = owner is None or owner == robot or not owner_live
            # The current order of choices is the robot's local preference.
            # A failed alignment is not exploration: never let its retry
            # penalty make a known corridor beat an unexplored side branch.
            score = (not bool(branch['visited_by']),
                     -branch.get('failed_by', {}).get(robot, 0), unclaimed)
            if self.goal is not None:
                dx = self.goal['x'] - node['x']
                dy = self.goal['y'] - node['y']
                toward_goal = (dx * math.cos(heading) +
                               dy * math.sin(heading))
            else:
                toward_goal = 0.0
            # Once a goal is known, reaching it matters more than keeping
            # robots on separate branches: the mapped way there wins, then
            # the heading. Before that, prefer a truly new branch, then one
            # that probably closes a loop, then the way to the nearest new one.
            distance = self._distance_via(node, branch, reach)
            if self.goal is not None:
                priority = (distance < math.inf, -distance, toward_goal, score)
            else:
                rank = (0 if branch['visited_by'] else
                        1 if self._likely_known(node, key) else 2)
                priority = (rank, -distance, score, toward_goal)
            candidates.append((not self._avoided(robot, node['id'], key, now),
                               priority, -len(candidates), option, key))

        if not candidates:
            for option in report['choices']:
                if not option.get('fallback', option['name'] == 'back'):
                    continue
                key = direction_key(float(option['heading']))
                branch = node['branches'].setdefault(
                    key, {'status': 'OPEN', 'owner': None, 'visited_by': []})
                if branch['status'] != 'BLOCKED':
                    candidates.append((True, (0,), 0, option, key))
        if not candidates:
            result = {'robot': robot, 'request_id': request_id,
                      'choice': None, 'junction_id': node['id'],
                      'junction_x': node['x'], 'junction_y': node['y'],
                      'reason': 'NO_OPEN_BRANCH'}
        else:
            _, priority, _, option, key = max(candidates, key=lambda c: c[:3])
            branch = node['branches'][key]
            owner = branch['owner']
            owner_live = (owner in self.robots and
                          now - self.robots[owner]['seen_at'] < PEER_TIMEOUT_SECONDS)
            shared = owner not in (None, robot) and owner_live
            if option.get('fallback', option['name'] == 'back'):
                reason = 'KNOWN_RETURN_ROUTE'
            elif self.goal is not None:
                reason = 'ROUTE_TO_GOAL' if priority[0] else 'TOWARD_GOAL'
            elif not branch['visited_by']:
                reason = ('UNEXPLORED_BRANCH' if priority[0] == 2
                          else 'LIKELY_KNOWN_BRANCH')
            else:
                reason = ('TOWARD_UNEXPLORED' if priority[1] > -math.inf
                          else 'REVISIT_OPEN_BRANCH')
            if not shared:
                branch['owner'] = robot
                branch['status'] = 'CLAIMED'
            if robot not in branch['visited_by']:
                branch['visited_by'].append(robot)
            self.trails[robot] = (node['id'], key)
            route_order = self.next_route_order
            self.next_route_order += 1
            result = {'robot': robot, 'request_id': request_id,
                      'choice': option['name'], 'junction_id': node['id'],
                      'heading': float(option['heading']),
                      'junction_x': node['x'], 'junction_y': node['y'],
                      'direction': key, 'shared': shared,
                      'reason': reason,
                      'route_order': route_order}
        result['branches'] = branch_details
        self.decisions[token] = result
        return result

    def route(self, from_node, to_node):
        """Shortest known path as [{'node', 'exit_dir', 'heading'}, ...]."""
        # One step per node before the destination; [] when already there,
        # None when the known, unblocked links do not reach it.
        if from_node not in self.junctions or to_node not in self.junctions:
            return None
        best = {from_node: 0.0}
        came_from = {}
        queue = [(0.0, from_node)]
        while queue:
            cost, node_id = heapq.heappop(queue)
            if node_id == to_node:
                break
            if cost > best[node_id]:
                continue
            node = self.junctions[node_id]
            for key, branch in node['branches'].items():
                link = branch.get('leads_to')
                if link is None or branch['status'] == 'BLOCKED':
                    continue
                other = self.junctions.get(link['node'])
                if other is None:
                    continue
                step = cost + math.hypot(other['x'] - node['x'], other['y'] - node['y'])
                if step < best.get(other['id'], math.inf):
                    best[other['id']] = step
                    came_from[other['id']] = (node_id, key)
                    heapq.heappush(queue, (step, other['id']))
        if to_node not in best:
            return None
        steps = []
        node_id = to_node
        while node_id != from_node:
            previous, key = came_from[node_id]
            branch = self.junctions[previous]['branches'][key]
            steps.append({'node': previous, 'exit_dir': key,
                          'heading': branch.get('heading', key * math.pi / 2)})
            node_id = previous
        return steps[::-1]

    def _live(self, robot, now):
        pose = self.robots.get(robot)
        return pose is not None and now - pose['seen_at'] < PEER_TIMEOUT_SECONDS

    def _hide_option(self, robot, other):
        """Nearest junction behind robot with a branch to wait in."""
        trail = self.trails.get(robot)
        pose = self.robots.get(robot)
        if trail is None or pose is None or trail[0] not in self.junctions:
            return None
        start, came_by = self.junctions[trail[0]], trail[1]
        best = {start['id']: math.hypot(pose['x'] - start['x'], pose['y'] - start['y'])}
        came_from = {start['id']: (None, came_by)}
        queue = [(best[start['id']], start['id'])]
        while queue:
            cost, node_id = heapq.heappop(queue)
            if cost > best[node_id]:
                continue
            node = self.junctions[node_id]
            arrival = came_from[node_id][1]
            if node['kind'] == 'JUNCTION':
                hide = self._hide_branch(node, arrival, robot)
                if hide is not None:
                    steps = []
                    walk = node_id
                    while came_from[walk][0] is not None:
                        previous, key = came_from[walk][0]
                        branch = self.junctions[previous]['branches'][key]
                        steps.append({'node': previous, 'exit_dir': key,
                                      'heading': branch.get('heading', key * math.pi / 2)})
                        walk = previous
                    heading = node['branches'][hide].get('heading', hide * math.pi / 2)
                    steps = steps[::-1] + [{'node': node_id, 'exit_dir': hide,
                                            'heading': heading}]
                    return {'distance': cost, 'route': steps, 'hide_node': node_id,
                            'hide_dir': hide, 'hide_heading': heading}
            for key, branch in node['branches'].items():
                link = branch.get('leads_to')
                if (link is None or branch['status'] == 'BLOCKED' or
                        (node_id == start['id'] and key == came_by)):
                    continue  # the corridor where the other robot is
                nxt = self.junctions.get(link['node'])
                if nxt is None:
                    continue
                step = cost + math.hypot(nxt['x'] - node['x'], nxt['y'] - node['y'])
                if step < best.get(nxt['id'], math.inf):
                    best[nxt['id']] = step
                    came_from[nxt['id']] = ((node_id, key), link['enter_dir'])
                    heapq.heappush(queue, (step, nxt['id']))
        return None

    def _hide_branch(self, node, arrival, robot):
        """Pick the branch to wait in: dead/explored, then new, then claimed."""
        ranked = []
        for key, branch in node['branches'].items():
            if key == arrival or branch.get('hidden_by') not in (None, robot):
                continue
            heading = branch.get('heading', key * math.pi / 2)
            occupied = False
            for name, pose in self.robots.items():
                dx, dy = pose['x'] - node['x'], pose['y'] - node['y']
                if (name != robot and math.hypot(dx, dy) < 0.5 and
                        abs(math.remainder(math.atan2(dy, dx) - heading, 2 * math.pi))
                        < math.pi / 4):
                    occupied = True
            if occupied:
                continue
            owner = branch['owner']
            claimed = owner not in (None, robot) and owner in self.robots
            if branch['status'] == 'BLOCKED' or (branch['visited_by'] and not claimed):
                rank = 0
            elif not claimed:
                rank = 1
            else:
                rank = 2
            ranked.append((rank, key))
        return min(ranked)[1] if ranked else None

    def _open_encounter(self, robot):
        return next((e for e in self.encounters.values()
                     if e['state'] != 'CLOSED' and robot in (e['yielder'], e['holder'])),
                    None)

    def head_on(self, robot, peer, now):
        """Two robots face each other on one line: order one to hide."""
        mine, theirs = self._open_encounter(robot), self._open_encounter(peer)
        if mine is not None and mine is theirs:
            return []  # already being handled
        escaping = [r for r, e in ((robot, mine), (peer, theirs))
                    if e is not None and e['yielder'] == r]
        if len(escaping) == 2:
            return []  # both already turning away from this corridor
        if escaping:
            # A robot already on its way to hide never turns round again.
            other = peer if escaping[0] == robot else robot
            option = self._hide_option(other, escaping[0])
            return self._start_encounter(other, None, option) if option else []
        options = {robot: self._hide_option(robot, peer),
                   peer: self._hide_option(peer, robot)}
        if options[robot] is None and options[peer] is None:
            return []
        if options[robot] is None or options[peer] is None:
            yielder = robot if options[robot] is not None else peer
        elif abs(options[robot]['distance'] - options[peer]['distance']) > 0.05:
            yielder = min(options, key=lambda r: options[r]['distance'])
        else:
            # Same as the old rule: the later entrant gives way.
            yielder = max(options, key=lambda r: (self.robots[r]['route_order'], r))
        holder = peer if yielder == robot else robot
        return self._start_encounter(yielder, holder, options[yielder])

    def _start_encounter(self, yielder, holder, option):
        encounter_id = self.next_encounter_id
        self.next_encounter_id += 1
        self.encounters[encounter_id] = {
            'id': encounter_id, 'yielder': yielder, 'holder': holder,
            'state': 'ESCAPING', 'hide_node': option['hide_node'],
            'hide_dir': option['hide_dir'], 'holder_trail': None}
        orders = [{'type': 'YIELD', 'robot': yielder, 'encounter': encounter_id,
                   'route': option['route'], 'hide_node': option['hide_node'],
                   'hide_dir': option['hide_dir'],
                   'hide_heading': option['hide_heading']}]
        if holder is not None:
            orders.append({'type': 'HOLD', 'robot': holder, 'encounter': encounter_id})
        return orders

    def hidden(self, robot):
        """Let the holder go once the yielder is parked in its branch."""
        encounter = next((e for e in self.encounters.values()
                          if e['state'] == 'ESCAPING' and e['yielder'] == robot), None)
        if encounter is None:
            return []
        encounter['state'] = 'HIDDEN'
        node = self.junctions[encounter['hide_node']]
        self._branch(node, encounter['hide_dir'])['hidden_by'] = robot
        self.trails[robot] = (node['id'], encounter['hide_dir'])
        holder = encounter['holder']
        if holder is None:
            encounter['holder_trail'] = None
            return []
        encounter['holder_trail'] = self.trails.get(holder)
        return [{'type': 'GO', 'robot': holder, 'encounter': encounter['id']}]

    def check_clear(self, now):
        """Release hidden robots once the robot they waited for is past."""
        orders = []
        for encounter in self.encounters.values():
            if encounter['state'] != 'HIDDEN':
                continue
            node = self.junctions[encounter['hide_node']]
            holder = encounter['holder']
            if holder is not None and self._live(holder, now):
                pose = self.robots[holder]
                if (self.trails.get(holder) == encounter['holder_trail'] or
                        math.hypot(pose['x'] - node['x'], pose['y'] - node['y']) <= 0.3):
                    continue
            branch = node['branches'].get(encounter['hide_dir'])
            if branch is not None:
                branch['hidden_by'] = None
            encounter['state'] = 'CLOSED'
            orders.append({'type': 'CLEAR', 'robot': encounter['yielder'],
                           'encounter': encounter['id']})
        return orders

    def release_branch(self, report, now):
        """Give a corridor up to the teammate met head-on inside it."""
        robot = report['robot']
        node = self.junctions.get(int(report['junction_id']))
        if node is None:
            return
        key = int(report['direction'])
        branch = node['branches'].get(key)
        if branch is not None and branch['owner'] == robot:
            branch['owner'] = None  # a teammate's shared claim stays theirs
            if branch['status'] != 'BLOCKED':
                branch['status'] = 'OPEN'  # visited_by stays: it is being explored
        peer = report.get('peer')
        self.avoid.setdefault(robot, {})[(node['id'], key)] = {
            'peer': peer, 'peer_trail': self.trails.get(peer), 'until': now + 60.0}

    def _avoided(self, robot, node_id, key, now):
        entry = self.avoid.get(robot, {}).get((node_id, key))
        if entry is None:
            return False
        if now > entry['until'] or self.trails.get(entry['peer']) != entry['peer_trail']:
            del self.avoid[robot][(node_id, key)]  # the teammate has moved on
            return False
        return True

    def _likely_known(self, node, key):
        """Tell whether a new arm points straight at a mapped node's open arm."""
        branch = node['branches'][key]
        if branch.get('leads_to') is not None:
            return False
        heading = branch.get('heading', key * math.pi / 2)
        back = direction_key(heading + math.pi)
        for other in self.junctions.values():
            if other['id'] == node['id'] or other['kind'] == 'START':
                continue
            dx, dy = other['x'] - node['x'], other['y'] - node['y']
            along = dx * math.cos(heading) + dy * math.sin(heading)
            lateral = abs(-dx * math.sin(heading) + dy * math.cos(heading))
            facing = other['branches'].get(back)
            if (0.0 < along <= 1.0 and lateral <= 0.10 and facing is not None
                    and facing.get('leads_to') is None):
                return True
        return False

    def _has_new_branch(self, node, robot, now):
        if node['kind'] != 'JUNCTION':
            return False
        for key, branch in node['branches'].items():
            owner = branch['owner']
            busy = owner not in (None, robot) and self._live(owner, now)
            if (branch['status'] != 'BLOCKED' and not branch['visited_by'] and not busy
                    and branch.get('hidden_by') is None
                    and not self._likely_known(node, key)):
                return True
        return False

    def _graph_distances(self, sources, robot, now):
        """Map distance from every node to the nearest of sources."""
        best = {source: 0.0 for source in sources}
        queue = [(0.0, source) for source in sources]
        heapq.heapify(queue)
        while queue:
            cost, node_id = heapq.heappop(queue)
            if cost > best[node_id]:
                continue
            node = self.junctions[node_id]
            for key, branch in node['branches'].items():
                link = branch.get('leads_to')
                if (link is None or branch['status'] == 'BLOCKED'
                        or self._avoided(robot, node_id, key, now)):
                    continue
                other = self.junctions.get(link['node'])
                if other is None:
                    continue
                step = cost + math.hypot(other['x'] - node['x'], other['y'] - node['y'])
                if step < best.get(other['id'], math.inf):
                    best[other['id']] = step
                    heapq.heappush(queue, (step, other['id']))
        return best

    def _distance_via(self, node, branch, reach):
        """Length of this branch plus the mapped distance beyond it."""
        link = branch.get('leads_to')
        other = self.junctions.get(link['node']) if link else None
        if other is None or other['id'] not in reach:
            return math.inf
        return (math.hypot(other['x'] - node['x'], other['y'] - node['y'])
                + reach[other['id']])

    def cancel_branch(self, report):
        """Release a junction choice when its robot has to yield before moving."""
        token = (report['robot'], report['request_id'])
        decision = self.decisions.pop(token, None)
        if decision is None or decision.get('direction') is None:
            return
        node = self.junctions.get(decision['junction_id'])
        if node is None:
            return
        branch = node['branches'].get(decision['direction'])
        if branch is None:
            return
        if branch['owner'] == report['robot'] and branch['status'] != 'BLOCKED':
            branch['owner'] = None
            branch['status'] = 'OPEN'
        if (report['robot'] in branch['visited_by'] and
                report['robot'] not in branch.get('arrived_by', [])):
            branch['visited_by'].remove(report['robot'])

    def block_branch(self, report):
        node = self.junctions.get(int(report['junction_id']))
        if node is None:
            return
        branch = node['branches'].get(int(report['direction']))
        if branch is not None:
            branch['status'] = 'BLOCKED'
            branch['owner'] = None

    def retry_branch(self, report):
        self.cancel_branch(report)
        node = self.junctions.get(int(report['junction_id']))
        if node is None:
            return
        branch = node['branches'].get(int(report['direction']))
        if branch is not None:
            failed = branch.setdefault('failed_by', {})
            failed[report['robot']] = failed.get(report['robot'], 0) + 1

    def set_goal(self, report):
        if self.goal is None:
            self.goal = {'robot': report['robot'],
                         'x': float(report['x']), 'y': float(report['y'])}
            node = self._junction(self.goal['x'], self.goal['y'], 'GOAL')
            if report.get('approach_heading') is not None:
                self._link(report['robot'], node, report['approach_heading'])

    def snapshot(self, now):
        return {
            'goal': self.goal,
            'robots': {
                name: {'x': p['x'], 'y': p['y'],
                       'heading': p['heading'], 'state': p['state'],
                       'route_order': p['route_order'],
                       'returning': p['returning'],
                       'yield_to': p['yield_to'],
                       'yield_junction': p['yield_junction'],
                       'yield_escape_heading': p['yield_escape_heading'],
                       'blocked_behind': p.get('blocked_behind', False)}
                for name, p in self.robots.items()
                if now - p['seen_at'] < PEER_TIMEOUT_SECONDS
            },
            'junctions': list(self.junctions.values()),
        }


class TeamCoordinator(Node):
    def __init__(self):
        super().__init__('maze_team_coordinator')
        self.run_id = uuid.uuid4().hex
        self.run_started_at = datetime.now(timezone.utc).isoformat()
        self.declare_parameter('junction_match_radius', MATCH_RADIUS_M)
        self.ledger = TeamLedger(
            float(self.get_parameter('junction_match_radius').value))
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.decision_pub = self.create_publisher(String, '/maze_team/decision', qos)
        self.state_pub = self.create_publisher(String, '/maze_team/state', qos)
        self.create_subscription(String, '/maze_team/report', self._on_report, 50)
        self.create_timer(0.5, self._publish_state)
        self.get_logger().info('Team coordinator ready: branch reservations and shared goal')

    def _publish_orders(self, orders):
        for order in orders:
            self._publish(self.decision_pub, order)
            self.get_logger().info(
                f"{order['type']} -> {order['robot']} (encounter {order['encounter']})")
        if orders:
            self._publish_state()

    def _publish(self, publisher, payload):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        publisher.publish(msg)

    def _on_report(self, msg):
        try:
            report = json.loads(msg.data)
            kind = report['type']
            now = time.monotonic()
            if kind == 'POSE':
                self.ledger.update_pose(report, now)
                self._publish_orders(self.ledger.check_clear(now))
            elif kind == 'HEAD_ON':
                self._publish_orders(
                    self.ledger.head_on(report['robot'], report['peer'], now))
            elif kind == 'HIDDEN':
                self._publish_orders(self.ledger.hidden(report['robot']))
            elif kind == 'RELEASE_BRANCH':
                self.ledger.release_branch(report, now)
                self._publish_state()
            elif kind == 'BRANCH_REQUEST':
                result = self.ledger.request_branch(report, now)
                self._publish(self.decision_pub, {
                    **result,
                    'run_id': self.run_id,
                    'run_started_at': self.run_started_at,
                })
                self.get_logger().debug(
                    f"{result['robot']} at J{result['junction_id']}: "
                    f"{result['choice']} (shared={result.get('shared', False)})")
                self._publish_state()
            elif kind == 'BLOCKED':
                self.ledger.block_branch(report)
                self._publish_state()
            elif kind == 'RETRY_BRANCH':
                self.ledger.retry_branch(report)
                self._publish_state()
            elif kind == 'CANCEL_BRANCH':
                self.ledger.cancel_branch(report)
                self._publish_state()
            elif kind == 'NODE':
                conflicts = len(self.ledger.link_conflicts)
                self.ledger.report_node(report, now)
                for line in self.ledger.link_conflicts[conflicts:]:
                    self.get_logger().warning(f'map link ignored: {line}')
                self._publish_state()
            elif kind == 'GOAL':
                self.ledger.set_goal(report)
                self._publish_state()
                self.get_logger().info(f"Goal reported by {report['robot']}")
        except (KeyError, ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'Ignoring malformed team report: {exc}')

    def _publish_state(self):
        self._publish(self.state_pub, {
            **self.ledger.snapshot(time.monotonic()),
            'run_id': self.run_id,
            'run_started_at': self.run_started_at,
        })


def main(args=None):
    rclpy.init(args=args)
    node = TeamCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
