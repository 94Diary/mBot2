"""Methods moved from team_coordinator.py; behavior is unchanged."""

import heapq
import math
from .config import (
    PEER_TIMEOUT_SECONDS,
    direction_key,
)


class GoalStateMixin:
    def goal_routes(self, now):
        """Tell each robot whether to turn back now that the goal is known."""
        goal = next((n for n in self.junctions.values() if n['kind'] == 'GOAL'), None)
        orders = []
        for robot, pose in self.robots.items():
            if (self.goal is None or robot == self.goal['robot']
                    or not self._live(robot, now) or pose['state'] == 'GOAL_REACHED'):
                continue
            turn_back = False
            trail = self.trails.get(robot)
            if goal is not None and not pose['returning'] and trail is not None:
                reach = self._graph_distances([goal['id']], robot, now)
                behind = self.junctions.get(trail[0])
                link = behind['branches'].get(trail[1], {}).get('leads_to') if behind else None
                ahead = self.junctions.get(link['node']) if link else None

                def via(node):
                    if node is None or node['id'] not in reach:
                        return math.inf
                    return (math.hypot(pose['x'] - node['x'], pose['y'] - node['y'])
                            + reach[node['id']])
                # Unknown corridor ahead: go back whenever the way behind is known.
                turn_back = via(behind) < via(ahead)
            orders.append({'type': 'GOAL_ROUTE', 'robot': robot, 'turn_back': turn_back})
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
        self.frontier_claims.pop(robot, None)

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

    def _graph_distances(self, sources, robot, now, skip_node=None):
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
                if other is None or other['id'] == skip_node:
                    continue
                if skip_node is not None and branch.get('hidden_by') not in (None, robot):
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
        previous_trail = self.request_trails.pop(token, None)
        if decision is None or decision.get('direction') is None:
            return
        if self.trails.get(report['robot']) == (decision['junction_id'], decision['direction']):
            self.frontier_claims.pop(report['robot'], None)
            # Asked ahead but never got there: the robot is still on its old leg.
            if previous_trail is None:
                self.trails.pop(report['robot'], None)
            else:
                self.trails[report['robot']] = previous_trail
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
                       'blocked_behind': p.get('blocked_behind', False),
                       'waiting_for': p.get('waiting_for'),
                       'junction_reached': name in self.junction_reached}
                for name, p in self.robots.items()
                if now - p['seen_at'] < PEER_TIMEOUT_SECONDS
            },
            'junctions': list(self.junctions.values()),
            'assignments': dict(self.assignments),
        }
