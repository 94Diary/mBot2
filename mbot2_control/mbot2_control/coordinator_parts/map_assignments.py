"""Methods moved from team_coordinator.py; behavior is unchanged."""

import math
from .config import (
    JUNCTION_REACHED_M,
    JUNCTION_PASSED_M,
    REASSIGN_MARGIN_M,
    direction_key,
)


class MapAssignmentsMixin:
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
            'waiting_for': report.get('waiting_for'),
            'route_order': route_order,
            'seen_at': now,
        }
        trail = self.trails.get(robot)
        node = self.junctions.get(trail[0]) if trail else None
        if node is not None and node['kind'] == 'JUNCTION':
            dx = self.robots[robot]['x'] - node['x']
            dy = self.robots[robot]['y'] - node['y']
            heading = node['branches'].get(trail[1], {}).get('heading')
            along = (dx * math.cos(heading) + dy * math.sin(heading)
                     if heading is not None else -1.0)
            lateral = (abs(-dx * math.sin(heading) + dy * math.cos(heading))
                       if heading is not None else math.inf)
            # A robot passing straight through may never report from the centre.
            if (math.hypot(dx, dy) <= JUNCTION_REACHED_M
                    or (0.0 < along <= JUNCTION_PASSED_M and lateral <= 0.10)):
                self.junction_reached.add(robot)
        self._update_assignments(now)

    def _is_task(self, node, key, robot, now):
        """Tell whether a branch is still unexplored work nobody has taken."""
        branch = node['branches'].get(key)
        if node['kind'] != 'JUNCTION' or branch is None:
            return False
        owner = branch['owner']
        busy = owner not in (None, robot) and self._live(owner, now)
        return (branch['status'] != 'BLOCKED' and branch.get('leads_to') is None
                and not branch.get('arrived_by') and not busy
                and branch.get('hidden_by') is None)

    def _busy(self, robot):
        """Tell whether a robot is out exploring a new branch it took."""
        trail = self.trails.get(robot)
        node = self.junctions.get(trail[0]) if trail else None
        branch = node['branches'].get(trail[1]) if node else None
        return (branch is not None and branch['owner'] == robot
                and branch.get('leads_to') is None
                and not self.robots[robot]['returning'])

    def _robot_reach(self, robot, target_id, now):
        """Measure the mapped distance to a node, back or ahead along its leg."""
        trail = self.trails.get(robot)
        pose = self.robots.get(robot)
        if trail is None or pose is None or trail[0] not in self.junctions:
            return math.inf
        reach = self._graph_distances([target_id], robot, now)
        behind = self.junctions[trail[0]]
        link = behind['branches'].get(trail[1], {}).get('leads_to')
        best = math.inf
        for node in (behind, self.junctions.get(link['node']) if link else None):
            if node is not None and node['id'] in reach:
                best = min(best, math.hypot(pose['x'] - node['x'], pose['y'] - node['y'])
                           + reach[node['id']])
        return best

    def _update_assignments(self, now):
        """Give idle robots the nearest unexplored branches, closest pairs first."""
        if self.goal is not None:
            self.assignments.clear()
            return
        idle = [r for r, p in self.robots.items()
                if self._live(r, now) and p['state'] not in ('GOAL_PARKING', 'GOAL_REACHED')
                and not self._busy(r)]
        tasks = {(n['id'], k) for n in self.junctions.values() for k in n['branches']
                 if self._is_task(n, k, None, now)}
        for robot, task in list(self.assignments.items()):
            if robot not in idle or (task['junction_id'], task['direction']) not in tasks:
                del self.assignments[robot]
        reach = {}

        def distance(robot, task):
            if (robot, task[0]) not in reach:
                reach[(robot, task[0])] = self._robot_reach(robot, task[0], now)
            return reach[(robot, task[0])]

        for _ in range(len(idle) * max(1, len(tasks)) + 1):
            taken = {(a['junction_id'], a['direction']): r
                     for r, a in self.assignments.items()}
            best = None
            for robot in idle:
                current = self.assignments.get(robot)
                mine = (math.inf if current is None else
                        distance(robot, (current['junction_id'], current['direction'])))
                for task in tasks:
                    if self._avoided(robot, *task, now):
                        continue
                    if any(other != robot and claim == task and self._live(other, now)
                           for other, claim in self.frontier_claims.items()):
                        continue
                    d = distance(robot, task)
                    holder = taken.get(task)
                    if current is not None:
                        # Already sent somewhere: switch only to a free task
                        # that is clearly closer than the one it has.
                        if holder is not None or d + REASSIGN_MARGIN_M > mine:
                            continue
                    # Take a reserved task only when clearly closer (0.5 m).
                    if d == math.inf or (holder is not None and
                                         d + REASSIGN_MARGIN_M > distance(holder, task)):
                        continue
                    if best is None or (d, robot, task) < best[:3]:
                        best = (d, robot, task, holder)
            if best is None:
                break
            _, robot, task, holder = best
            if holder is not None:
                del self.assignments[holder]
            self.assignments.pop(robot, None)
            self.assignments[robot] = {'junction_id': task[0], 'direction': task[1]}

    def _junction(self, x, y, kind='JUNCTION'):
        if kind in ('START', 'GOAL'):
            # One of each, never merged with a junction next to it: robots
            # leave the start box and first see the goal area at different
            # points, sometimes only centimetres from a junction.
            same = next((n for n in self.junctions.values() if n['kind'] == kind), None)
            if same is not None:
                return same
        else:
            for node in self.junctions.values():
                if (node['kind'] not in ('START', 'GOAL') and
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
