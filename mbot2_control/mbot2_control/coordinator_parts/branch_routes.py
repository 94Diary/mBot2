"""Methods moved from team_coordinator.py; behavior is unchanged."""

import heapq
import math
from .config import (
    PEER_TIMEOUT_SECONDS,
    direction_key,
)


class BranchRoutesMixin:
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

    def _exploration_choices(self, node, choices, robot, now, queued):
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
                if task not in targets or option.get('fallback', option['name'] == 'back'):
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
        choices = [(option, direction_key(float(option['heading'])))
                   for option in report['choices']]
        for option, key in choices:
            self._branch(node, key, option['heading'])
        self._update_assignments(now)
        if self.goal is not None:
            goal_node = next((n for n in self.junctions.values()
                              if n['kind'] == 'GOAL'), None)
            reach = (self._graph_distances([goal_node['id']], robot, now)
                     if goal_node is not None else {})
        for option, key in choices:
            branch = node['branches'][key]
            branch_details.append({
                'name': option['name'], 'status': branch['status'],
                'visited_by': list(branch['visited_by']),
                'fallback': bool(option.get('fallback', option['name'] == 'back')),
                'failed_attempts': branch.get('failed_by', {}).get(robot, 0),
                'endpoint_known': branch.get('leads_to') is not None,
            })
            if self.goal is None:
                continue
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
            dx = self.goal['x'] - node['x']
            dy = self.goal['y'] - node['y']
            toward_goal = dx * math.cos(heading) + dy * math.sin(heading)
            distance = self._distance_via(node, branch, reach)
            priority = (distance < math.inf, -distance, toward_goal, score)
            candidates.append((not self._avoided(robot, node['id'], key, now),
                               priority, -len(candidates), option, key,
                               'ROUTE_TO_GOAL' if priority[0] else 'TOWARD_GOAL', None))

        queued = set()
        if self.goal is None:
            queued = self._robots_queued_behind(robot, now)
            for score, option, key, reason, task in self._exploration_choices(
                    node, choices, robot, now, queued):
                candidates.append((True, score, -len(candidates), option, key, reason, task))
        elif not candidates:
            for option in report['choices']:
                if not option.get('fallback', option['name'] == 'back'):
                    continue
                key = direction_key(float(option['heading']))
                branch = node['branches'].setdefault(
                    key, {'status': 'OPEN', 'owner': None, 'visited_by': []})
                if branch['status'] != 'BLOCKED':
                    candidates.append((True, (0,), 0, option, key, 'KNOWN_RETURN_ROUTE', None))
        if not candidates:
            result = {'robot': robot, 'request_id': request_id,
                      'choice': None, 'junction_id': node['id'],
                      'junction_x': node['x'], 'junction_y': node['y'],
                      'reason': 'WAIT_FOR_FRONTIER' if self.goal is None else 'NO_OPEN_BRANCH'}
        else:
            _, _, _, option, key, reason, task = max(candidates, key=lambda c: c[:3])
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
                if transferred:
                    reason = 'TAKE_QUEUED_TASK'
                if task == (node['id'], key):
                    self.assignments.pop(robot, None)
                else:
                    self.frontier_claims[robot] = task
                    if transferred:
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
