# C1 Shared Team Map Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The coordinator builds a node/edge map from robot reports, can compute shortest routes, and robots skip the in-place scan at nodes the team already knows.

**Architecture:** `TeamLedger.junctions` becomes a node table (kinds START/JUNCTION/CORNER/DEAD_END/GOAL) whose branches carry `heading` and `leads_to`; a per-robot `trail` links each newly reached node to the branch the robot left by. Robots send a new `NODE` report for start/corner/dead end and keep `BRANCH_REQUEST` for junctions. In black mode, `_start_junction_scan` looks the pivot up in the team map first and, if known, feeds the stored exits straight into `_finish_junction_scan`.

**Tech Stack:** ROS 2 Humble rclpy, unittest/pytest, Gazebo Fortress.

**Spec:** `docs/superpowers/specs/2026-10-30-c1-shared-map-design.md`

## Global Constraints
- Node matching in the sim uses world position within `junction_match_radius` (0.25 m on seed7). Counting/re-anchoring is out of scope; it lives in `toRealbot.md`.
- `TeamLedger` keeps no ROS imports.
- Existing behaviour without team data (solo mode, stale `/maze_team/state`, yellow/green modes) is unchanged.
- No speed or gain tuning.
- Any new sim-vs-real finding is appended to `toRealbot.md`.

## Review Focus
1. A robot backing through a node while yielding must not overwrite an existing link (first link kept) — Task 1 `test_conflicting_link_keeps_first`.
2. START reports from robots that found the line at different x must all map to one START node — Task 1 `test_all_start_reports_share_one_node`.
3. A known JUNCTION reached from a different side than its first visit must offer every recorded branch except the inbound one — Task 4 `test_known_junction_from_other_side_offers_other_branches`.
4. A CORNER later found to have a third branch must become a JUNCTION, never the other way round — Task 1 `test_corner_upgrades_to_junction`.
5. Stale team state must fall back to a real scan — Task 4 `test_stale_team_map_scans`.

---

### Task 1: Ledger map — nodes, branch links, trail, NODE report

**Files:**
- Modify: `mbot2_control/mbot2_control/team_coordinator.py`
- Create: `mbot2_control/test/test_team_map.py`

**Interfaces:**
- Produces:
  - node dict: `{'id', 'kind', 'x', 'y', 'branches'}`; branch dict gains `'heading'` (float or absent) and `'leads_to'` (`{'node': int, 'enter_dir': int}` or `None`).
  - `TeamLedger.trails: dict[robot, (node_id, exit_dir)]`, `TeamLedger.link_conflicts: list[str]`.
  - `TeamLedger.report_node(report: dict, now: float) -> dict` (returns the node). Report keys: `robot, kind ('START'|'CORNER'|'DEAD_END'), x, y, approach_heading`, plus `heading` for START and `exit_heading` for CORNER.
  - `request_branch` also sets node `kind='JUNCTION'`, links via `approach_heading`, records each choice's `heading` on first sight, and sets the trail exit to the chosen key.
  - `set_goal(report)` creates a GOAL node (x, y) and links it via the trail when `report` has `approach_heading`.
  - `TeamCoordinator._on_report` routes `'NODE'` to `report_node` and logs new entries of `link_conflicts` as warnings.

- [ ] **Step 1: Write the failing tests** in `test/test_team_map.py` (plain `unittest`, `TeamLedger(match_radius=0.25)`), helper `walk(ledger, robot, kind, x, y, approach, **extra)` that calls `report_node` or `request_branch` with a single choice:
  - `test_straight_walk_links_both_ends`: START (−1.0, 0) heading 0 → JUNCTION (0, 0) approach 0, choose straight (heading 0) → JUNCTION (0.5, 0) approach 0. Assert J1 branch `direction_key(0)` `leads_to == {'node': J2, 'enter_dir': direction_key(pi)}` and J2 branch `direction_key(pi)` leads back to J1 with `enter_dir == direction_key(0)`; START's east branch leads to J1.
  - `test_corner_sits_between_junctions`: J1 (0,0) choose east → CORNER (0.5,0) approach 0 exit_heading pi/2 → JUNCTION (0.5,0.5) approach pi/2. Assert CORNER `kind == 'CORNER'` and links to both.
  - `test_dead_end_node_and_blocked_branch`: J1 choose east → `block_branch` for that choice → DEAD_END (0.4,0) approach 0. Assert node kind `DEAD_END`, J1 east branch `status == 'BLOCKED'` and `leads_to['node']` is the dead end.
  - `test_two_robots_reach_same_junction_from_different_sides`: mbot1 reaches (0,0) approach 0, mbot2 reaches (0.1,0) approach pi. Same node id; west and east branches link to each robot's previous node.
  - `test_conflicting_link_keeps_first`: link J1 east → J2, then a second robot's trail claims J1 east → J3. J1 east still leads to J2; `len(link_conflicts) == 1`.
  - `test_all_start_reports_share_one_node`: START at x = −1.85, −2.10, −2.38 → one START node, position of the first.
  - `test_corner_upgrades_to_junction`: CORNER at (0,0), later `request_branch` at (0.05,0) → kind `JUNCTION`; a later CORNER report at the same spot keeps `JUNCTION`.
  - `test_goal_becomes_node`: trail at J1 east, `set_goal({'robot', 'x': 0.5, 'y': 0, 'approach_heading': 0})` → node kind `GOAL` linked to J1 east.

- [ ] **Step 2: Run** `cd ~/ros2_ws/src/mbot2_control && python3 -m pytest -q test/test_team_map.py` — Expected: FAIL (`report_node` missing).

- [ ] **Step 3: Implement** in `team_coordinator.py`: node kind on creation (`_junction(x, y, kind)`), JUNCTION never downgraded; one START node (kind lookup, no radius); `_link(robot, node, approach_heading)` writes both ends only where `leads_to` is `None` and records a conflict string otherwise, never links a node to itself; `report_node`; trail updates per spec (START: `heading`, CORNER: `exit_heading`, DEAD_END: `approach_heading + pi`, junction: chosen key); `_on_report` routing.

- [ ] **Step 4: Run** `python3 -m pytest -q test/test_team_map.py test/test_team_coordinator.py` — Expected: all new pass; only the baseline `test_stale_claim_can_be_reused` fails.

- [ ] **Step 5: Commit** `feat: team ledger links nodes into a shared map` (ask the user first).

### Task 2: Shortest route

**Files:**
- Modify: `mbot2_control/mbot2_control/team_coordinator.py`
- Test: `mbot2_control/test/test_team_map.py`

**Interfaces:**
- Consumes: Task 1 node/branch dicts.
- Produces: `TeamLedger.route(from_node: int, to_node: int) -> list[dict] | None`, each step `{'node': int, 'exit_dir': int, 'heading': float}` for every node before the destination; `[]` when `from_node == to_node`; `None` when unreachable. Dijkstra (`heapq`) over `leads_to` edges, weight = straight distance between node positions, skipping `BLOCKED` branches; step `heading` is the branch `heading` or `exit_dir * pi / 2`.

- [ ] **Step 1: Write the failing tests**: `test_route_prefers_shorter_side_of_loop` (square loop J1(0,0)–J2(1,0)–J3(1,1)–J4(0,1) plus a long detour; route J1→J3 has 2 steps), `test_route_avoids_blocked_branch`, `test_route_unreachable_is_none`, `test_route_to_self_is_empty`.
- [ ] **Step 2: Run** `python3 -m pytest -q test/test_team_map.py -k route` — Expected: FAIL (`route` missing).
- [ ] **Step 3: Implement** `route`.
- [ ] **Step 4: Run** `python3 -m pytest -q test/test_team_map.py` — Expected: PASS.
- [ ] **Step 5: Commit** `feat: shortest route over the team map` (ask first).

### Task 3: Robot reports START, CORNER, DEAD_END and GOAL heading

**Files:**
- Modify: `mbot2_control/mbot2_control/maze_solver.py`
- Test: `mbot2_control/test/test_junction_scan.py`

**Interfaces:**
- Produces: `self.scan_pivot_world` (world axle point recorded in `_start_junction_scan`); reports `{'type': 'NODE', 'kind': ..., 'x', 'y', 'approach_heading', ...}` via `_team_report`; GOAL report gains `'approach_heading'` (current world yaw).
- Sends: START when SEARCH first sees black (x, y = world sensor position, `heading` = world yaw, `approach_heading` = world yaw); CORNER from `_finish_junction_scan` when the single exit is left or right (not straight-only false triggers), `exit_heading` = world h0 + θ; DEAD_END from `_finish_junction_scan` with no exits and from the wall branch next to its `BLOCKED` report (x, y = world pose there, `approach_heading` = world yaw).

- [ ] **Step 1: Write the failing tests** (FloorHarness, team mode): `test_new_corner_is_reported` (INBOUND+LEFT → one `NODE` with `kind == 'CORNER'` and `exit_heading` within 0.35 of pi/2), `test_false_trigger_is_not_reported` (speck test floor → no `NODE` CORNER), `test_line_end_reports_dead_end` (INBOUND only → `NODE` DEAD_END), `test_leaving_start_box_reports_start` (state SEARCH on a line → `NODE` START).
- [ ] **Step 2: Run** `python3 -m pytest -q test/test_junction_scan.py` — Expected: the 4 new tests FAIL.
- [ ] **Step 3: Implement** the reports.
- [ ] **Step 4: Run** `python3 -m pytest -q test/` — Expected: only the 3 baseline failures (flake8 count ≤ 10, pep257 ≤ 22, `test_stale_claim_can_be_reused`).
- [ ] **Step 5: Commit** `feat: robots report start, corners and dead ends to the team map` (ask first).

### Task 4: Skip the scan at known nodes

**Files:**
- Modify: `mbot2_control/mbot2_control/maze_solver.py`
- Test: `mbot2_control/test/test_junction_scan.py`

**Interfaces:**
- Consumes: `/maze_team/state['junctions']` (Task 1 node dicts).
- Produces: `self.team_nodes: list[dict]` stored in `_on_team_state`; `_known_node(world_xy) -> dict | None` returns the nearest node of kind JUNCTION or CORNER within 0.25 m, only while `now - last_team_state_at <= 3.0`.
- Behaviour: in `_start_junction_scan` (black mode, team enabled), after setting the pivot, if `_known_node(scan_pivot_world)` returns a node: fill `self.scan_hits` from its branches (θ = `angle_difference(branch heading, world h0)`, binned with the existing windows; branches outside the windows, i.e. the inbound one, are ignored), log `ใช้ข้อมูลทีมที่ J<id>`, and call `_finish_junction_scan()` without rotating. Branch heading = `branch['heading']` if present else `int(key) * pi / 2`.

- [ ] **Step 1: Write the failing tests**: `test_known_junction_is_not_scanned` (team_nodes holds J at (0.5,0) with branches west/east/north; drive INBOUND+STRAIGHT+LEFT; scan rotation < 0.1 rad and the `BRANCH_REQUEST` choices are straight+left+back), `test_known_junction_from_other_side_offers_other_branches` (approach J from the north arm: choices exclude north), `test_known_corner_turns_without_scan` (CORNER node; robot turns to its exit, sends `NODE` CORNER, scan rotation < 0.1 rad beyond the exit turn), `test_stale_team_map_scans` (same node but `last_team_state_at` 5 s old → normal sweep).
- [ ] **Step 2: Run** `python3 -m pytest -q test/test_junction_scan.py` — Expected: the 4 new tests FAIL.
- [ ] **Step 3: Implement** as described.
- [ ] **Step 4: Run** `python3 -m pytest -q test/` — Expected: only the 3 baseline failures.
- [ ] **Step 5: Commit** `feat: skip the junction scan where the team map already knows the exits` (ask first).

### Task 5: Docs and sim verification

**Files:**
- Modify: `mbot2_control/MULTI_ROBOT.md` (NODE report, node kinds, `leads_to`, `route`), `toRealbot.md` (anything new found in the sim)
- Scratch (not committed): a script that reads the black line boxes from `mbot2_seed7_maze.sdf`, builds the true node graph on the 0.5 m grid, and compares it with the last `/maze_team/state` snapshot.

- [ ] **Step 1:** Announce to the user, check no other Gazebo runs, rebuild with `--symlink-install`, assert the installed `maze_solver` source contains `_known_node` and `team_coordinator` contains `report_node`.
- [ ] **Step 2:** Run the 3-robot headless sim for 600 s with `/maze_team/state` recorded. Expected: goal found ≤ about 3:46 (±30 s), all 3 `GOAL_REACHED` ≤ about 5:05 (±30 s), `ใช้ข้อมูลทีม` count > 0.
- [ ] **Step 3:** Run the comparison script. Expected: every recorded link matches a real line connection; list any mismatch or missed corner.
- [ ] **Step 4:** Update the docs; report results to the user; commit `docs: shared team map` (ask first).
