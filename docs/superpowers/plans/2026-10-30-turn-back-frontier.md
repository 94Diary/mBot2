# Turn-back and Frontier Steering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** In team mode a robot that meets a teammate head-on and lacks priority turns round and drives forward back instead of reversing, releasing its branch; the coordinator steers every robot toward the nearest unexplored branch (or along the mapped shortest way to the goal) and keeps released corridors away from the robot that gave them up.

**Architecture:** `TeamLedger.release_branch` + per-robot `avoid` set; `request_branch` ranks options (new → likely-known → toward nearest frontier / map distance to goal). `MazeSolver` gets `head_on_hiding` (default False) selecting the C4 path or the new `_begin_turn_back`.

**Tech Stack:** ROS 2 Humble rclpy, unittest/pytest, Gazebo Fortress.

**Spec:** `docs/superpowers/specs/2026-10-30-turn-back-frontier-design.md`

## Global Constraints
- Values: avoid expires after 60 s or when the peer's trail changes; likely-known: within 1.0 m ahead, ≤ 0.10 m lateral, facing branch unlinked.
- Reasons: `UNEXPLORED_BRANCH`, `LIKELY_KNOWN_BRANCH`, `TOWARD_UNEXPLORED`, `ROUTE_TO_GOAL`, old ones kept.
- No reversing yields in team mode with `head_on_hiding` false; C4 unchanged when true.
- No speed/gain tuning; findings to `toRealbot.md`.

## Review Focus
1. Both robots of a pair must agree who turns (same `_peer_has_priority` on both sides) — Task 3 `test_later_robot_turns_back` + `test_earlier_robot_waits`.
2. A turned-back robot must not be sent straight back into the released corridor while another branch exists — Task 1 `test_released_branch_avoided`.
3. Frontier steering must ignore BLOCKED branches and avoided ones — Task 2 `test_frontier_route_skips_blocked`.
4. Goal steering falls back to the heading rule when the goal is not on the map — Task 2 `test_goal_not_on_map_uses_heading`.
5. Turning back while already RETURNING must not leave `returning` True — Task 3 `test_returning_robot_turns_back_into_follow`.

---

### Task 1: Release and avoid
**Files:** `team_coordinator.py`; Create `test/test_team_frontier.py`
**Produces:** `TeamLedger.release_branch(report: dict, now: float) -> None` (owner None, OPEN unless BLOCKED, keep `visited_by`, `self.avoid[robot][(junction_id, direction)] = {'peer', 'peer_trail', 'until': now + 60}`); `_avoided(robot, node_id, key, now) -> bool` (drops expired entries / peer trail changed); `request_branch` skips avoided keys unless nothing else is left; wrapper routes `RELEASE_BRANCH`.
- [ ] Tests: `test_release_keeps_visited`, `test_released_branch_avoided`, `test_avoided_branch_used_when_nothing_else`, `test_avoid_cleared_when_peer_moves_on`. Run → FAIL; implement; run `test_team_frontier.py test_team_map.py test_team_coordinator.py` → only baseline fail.

### Task 2: Ranking — likely-known, frontier, goal by map
**Files:** `team_coordinator.py`; Test `test/test_team_frontier.py`
**Produces:** `_likely_known(node, key) -> bool`; `_frontier_distance(node, key, robot, now) -> float` (edge to `leads_to` + multi-source Dijkstra distance to the nearest JUNCTION with a truly new branch; `inf` if none/unknown link); `_goal_distance(node, key, robot, now) -> float | None` (map distance to the GOAL node; None if goal not mapped/reachable). Pre-goal priority tuple: `(rank, -frontier_distance, existing score…)` with rank 2 new, 1 likely-known, 0 visited; post-goal: map distance when any option has one, else old heading rule.
- [ ] Tests: `test_likely_known_ranks_below_new`, `test_all_visited_heads_to_nearest_frontier` (reason `TOWARD_UNEXPLORED`), `test_frontier_route_skips_blocked`, `test_goal_by_map_distance` (`ROUTE_TO_GOAL`, avoided branch skipped), `test_goal_not_on_map_uses_heading`. Run → FAIL; implement; run suite → only baseline fails.

### Task 3: Robot turns back
**Files:** `maze_solver.py`; Test `test/test_junction_scan.py`
**Produces:** parameter `head_on_hiding` (bool, default False); `_begin_turn_back(peer: str)`; head-on handling: hiding → existing `_on_head_on`, else priority peer → `_begin_turn_back`, non-priority → continue (ultrasonic stop does the waiting); obstacle block: opposite peer → same; `LINE_LOST` peer → wait only; follower chain-yield rule disabled when hiding is off. Existing C4 tests set `self.node.head_on_hiding = True`.
- [ ] Tests: `test_later_robot_turns_back` (UTURN target ≈ +π, RETURNING after the turn, `RELEASE_BRANCH` with the active branch and `peer`, no `BLOCKED`, no command with `linear.x < -0.01`), `test_earlier_robot_waits` (no turn, stops by ultrasonic 0.2), `test_lost_peer_ahead_waits`, `test_returning_robot_turns_back_into_follow`. Run → FAIL; implement; run suite → only baseline fails, lint ≤ 10 / 22.

### Task 4: Docs, sim, review
- [ ] `MULTI_ROBOT.md` (rules, reasons, parameter), `toRealbot.md`.
- [ ] Announce, check no other Gazebo, rebuild `--symlink-install`, assert installed sources contain `_begin_turn_back` and `release_branch`.
- [ ] 600 s 3-robot sim with state recording. Expected: all 3 `GOAL_REACHED`, `ถอยให้` count 0, no face-to-face freeze > 30 s; report counts of `กลับตัวให้`, `TOWARD_UNEXPLORED`, `ROUTE_TO_GOAL`.
- [ ] Final review (fresh reviewer), fix Critical/Important by TDD, ask the user before committing (C4 code + this work).
