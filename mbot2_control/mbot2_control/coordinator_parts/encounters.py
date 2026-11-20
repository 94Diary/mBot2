"""Methods moved from team_coordinator.py; behavior is unchanged."""

import heapq
import math
from .config import (
    PEER_TIMEOUT_SECONDS,
)


class EncountersMixin:
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
