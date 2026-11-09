"""Methods moved from team_coordinator.py; behavior is unchanged."""

import heapq
import math
from .config import (
    PEER_TIMEOUT_SECONDS,
    direction_key,
)


class BranchRoutesMixin:
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
        self._update_assignments(now)
        task = self.assignments.get(robot)
        task_reach = (self._graph_distances([task['junction_id']], robot, now)
                      if task is not None else {})
        steering = []
        if self.goal is not None:
            goal_node = next((n for n in self.junctions.values()
                              if n['kind'] == 'GOAL'), None)
            reach = (self._graph_distances([goal_node['id']], robot, now)
                     if goal_node is not None else {})
        else:
            self.frontier_claims.pop(robot, None)
            frontier = [n['id'] for n in self.junctions.values()
                        if n['id'] != node['id'] and self._has_new_branch(n, robot, now)]
            # Leave a frontier a teammate is already heading for, if there
            # is another one; otherwise share it.
            claimed = {target for other, target in self.frontier_claims.items()
                       if other != robot and self._live(other, now)}
            free = [n for n in frontier if n not in claimed]
            reach, origin = self._graph_distances(free or frontier, robot, now,
                                                  with_origin=True)
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
            if task is not None:
                at_task = (node['id'] == task['junction_id'] and key == task['direction'])
                steering.append((not self._avoided(robot, node['id'], key, now),
                                 (at_task, -self._distance_via(node, branch, task_reach),
                                  priority),
                                 -len(steering), option, key))
            candidates.append((not self._avoided(robot, node['id'], key, now),
                               priority, -len(candidates), option, key))
        # A reserved branch the map can reach steers the choice (C2).
        if steering and any(s[0] and (s[1][0] or s[1][1] > -math.inf) for s in steering):
            candidates = steering
        else:
            task = None

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
            elif task is not None:
                reason = 'ASSIGNED_BRANCH' if priority[0] else 'TOWARD_ASSIGNED'
                if priority[0]:
                    del self.assignments[robot]  # it starts exploring now
            elif self.goal is not None:
                reason = 'ROUTE_TO_GOAL' if priority[0] else 'TOWARD_GOAL'
            elif not branch['visited_by']:
                reason = ('UNEXPLORED_BRANCH' if priority[0] == 2
                          else 'LIKELY_KNOWN_BRANCH')
            else:
                reason = ('TOWARD_UNEXPLORED' if priority[1] > -math.inf
                          else 'REVISIT_OPEN_BRANCH')
                link = branch.get('leads_to')
                if reason == 'TOWARD_UNEXPLORED' and link is not None:
                    self.frontier_claims[robot] = origin.get(link['node'])
            if not shared:
                branch['owner'] = robot
                branch['status'] = 'CLAIMED'
            if robot not in branch['visited_by']:
                branch['visited_by'].append(robot)
            self.request_trails[token] = self.trails.get(robot)
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
