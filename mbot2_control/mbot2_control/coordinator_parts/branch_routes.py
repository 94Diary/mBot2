"""Methods moved from team_coordinator.py; behavior is unchanged."""

import heapq
import math
from .config import (
    PEER_TIMEOUT_SECONDS,
    direction_key,
)


class BranchRoutesMixin:
    def _goal_choices(self, node, choices, robot, now):
        """Offer only exits with a usable path to the goal."""
        goal = next((n for n in self.junctions.values() if n['kind'] == 'GOAL'), None)
        if goal is None:
            return
        node_id = node['id']
        for option, key in choices:
            branch = node['branches'][key]
            link = branch.get('leads_to')
            if (link is None or branch['status'] == 'BLOCKED'
                    or branch.get('hidden_by') not in (None, robot)
                    or self._avoided(robot, node_id, key, now)):
                continue
            neighbor = self.junctions.get(link['node'])
            if neighbor is None or neighbor['id'] == node_id:
                continue
            distances = self._graph_distances([neighbor['id']], robot, now, skip_node=node_id)
            if goal['id'] in distances:
                distance = math.hypot(neighbor['x'] - node['x'], neighbor['y'] - node['y'])
                yield distance + distances[goal['id']], option, key

    def _robots_queued_behind(self, robot, now):
        """Find live robots waiting directly or indirectly for this queue leader."""
        if self._live(self.robots.get(robot, {}).get('waiting_for'), now):
            return set()
        followers = {}
        for name, pose in self.robots.items():
            ahead = pose.get('waiting_for')
            if ahead is not None and self._live(name, now):
                followers.setdefault(ahead, []).append(name)
        queued = {robot}
        pending = [robot]
        while pending:
            for follower in followers.get(pending.pop(), []):
                if follower not in queued:
                    queued.add(follower)
                    pending.append(follower)
        return queued - {robot}

    def _exploration_choices(self, node, choices, robot, now, queued, inbound_key):
        """Rank only branches with reachable, available exploration work."""
        node_id = node['id']
        self.frontier_claims.pop(robot, None)
        # Only active branch owners can keep a robot already at the branch waiting.
        reserved = {claim for other, claim in self.frontier_claims.items()
                    if other != robot and other not in queued and self._live(other, now)
                    and claim[0] != node_id}
        reserved.update((task['junction_id'], task['direction'])
                        for other, task in self.assignments.items()
                        if other != robot and other not in queued and self._live(other, now)
                        and task['junction_id'] != node_id)
        assigned = self.assignments.get(robot)
        assigned_key = ((assigned['junction_id'], assigned['direction'])
                        if assigned is not None else None)
        targets = {}
        for target in self.junctions.values():
            for key, branch in target['branches'].items():
                task = (target['id'], key)
                if (task not in reserved and self._is_task(target, key, robot, now)
                        and not self._avoided(robot, *task, now)):
                    targets[task] = branch

        for option, key in choices:
            branch = node['branches'][key]
            if (branch['status'] == 'BLOCKED' or self._avoided(robot, node_id, key, now)
                    or branch.get('hidden_by') not in (None, robot)):
                continue
            link = branch.get('leads_to')
            if link is None:
                task = (node_id, key)
                if (task not in targets or key == inbound_key
                        or option.get('fallback', option['name'] == 'back')):
                    continue
                likely = self._likely_known(node, key)
                rank = 1 if likely else 2
                reason = 'LIKELY_KNOWN_BRANCH' if likely else 'UNEXPLORED_BRANCH'
                if task == assigned_key:
                    reason = 'ASSIGNED_BRANCH'
                yield (task == assigned_key, rank, 0.0,
                       -branch.get('failed_by', {}).get(robot, 0)), option, key, reason, task
                continue

            neighbor = self.junctions.get(link['node'])
            if neighbor is None or neighbor['id'] == node_id:
                continue
            # Never count a route that needs to return through this junction.
            distances = self._graph_distances([neighbor['id']], robot, now, skip_node=node_id)
            first_leg = math.hypot(neighbor['x'] - node['x'], neighbor['y'] - node['y'])
            for task, target_branch in targets.items():
                if task[0] not in distances:
                    continue
                reason = 'TOWARD_ASSIGNED' if task == assigned_key else 'TOWARD_UNEXPLORED'
                score = (task == assigned_key, 0, -first_leg - distances[task[0]],
                         not target_branch['visited_by'])
                yield score, option, key, reason, task

    def request_branch(self, report, now):
        robot = report['robot']
        request_id = report['request_id']
        token = (robot, request_id)
        if token in self.decisions:
            return self.decisions[token]

        node = self._junction(float(report['x']), float(report['y']))
        inbound_key = None
        if report.get('approach_heading') is not None:
            self._link(robot, node, report['approach_heading'])
        # An arrival alone does not prove where the corridor leads.
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
        choices = [(option, direction_key(float(option['heading'])))
                   for option in report['choices']]
        for option, key in choices:
            branch = self._branch(node, key, option['heading'])
            if key != inbound_key and not option.get('fallback', option['name'] == 'back'):
                branch['observed_exit'] = True
        self._update_assignments(now)
        for option, key in choices:
            branch = node['branches'][key]
            branch_details.append({
                'name': option['name'], 'status': branch['status'],
                'visited_by': list(branch['visited_by']),
                'fallback': bool(option.get('fallback', option['name'] == 'back')),
                'failed_attempts': branch.get('failed_by', {}).get(robot, 0),
                'endpoint_known': branch.get('leads_to') is not None,
            })

        queued = set()
        if self.goal is not None:
            self.frontier_claims.pop(robot, None)
            for distance, option, key in self._goal_choices(node, choices, robot, now):
                candidates.append(((-distance,), -len(candidates), option, key,
                                   'ROUTE_TO_GOAL', None))
        if not candidates:
            queued = self._robots_queued_behind(robot, now)
            for score, option, key, reason, task in self._exploration_choices(
                    node, choices, robot, now, queued, inbound_key):
                if self.goal is not None:
                    reason = 'EXPLORE_GOAL_CONNECTION'
                candidates.append((score, -len(candidates), option, key, reason, task))
        if not candidates:
            result = {'robot': robot, 'request_id': request_id,
                      'choice': None, 'junction_id': node['id'],
                      'junction_x': node['x'], 'junction_y': node['y'],
                      'reason': ('WAIT_FOR_FRONTIER' if self.goal is None
                                 else 'WAIT_FOR_GOAL_ROUTE')}
        else:
            _, _, option, key, reason, task = max(candidates, key=lambda c: c[:2])
            branch = node['branches'][key]
            owner = branch['owner']
            owner_live = (owner in self.robots and
                          now - self.robots[owner]['seen_at'] < PEER_TIMEOUT_SECONDS)
            shared = owner not in (None, robot) and owner_live
            if task is not None:
                assignment = {'junction_id': task[0], 'direction': task[1]}
                transferred = False
                for follower in queued:
                    if self.frontier_claims.get(follower) == task:
                        del self.frontier_claims[follower]
                        transferred = True
                    if self.assignments.get(follower) == assignment:
                        del self.assignments[follower]
                        transferred = True
                if transferred and self.goal is None:
                    reason = 'TAKE_QUEUED_TASK'
                if task == (node['id'], key):
                    self.assignments.pop(robot, None)
                else:
                    self.frontier_claims[robot] = task
                    if transferred and self.goal is None:
                        self.assignments[robot] = assignment
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
        result['routing_revision'] = self.routing_revision()
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
