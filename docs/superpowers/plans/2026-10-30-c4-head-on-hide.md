# C4 Head-on Hiding Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** When two robots meet head-on, the coordinator picks the one with the shorter way back to a hiding branch; it turns round, follows the mapped route forward, hides 0.25 m into a side branch, and resumes after the other robot has passed.

**Architecture:** `TeamLedger` gains encounter bookkeeping (`head_on`, `hidden`, `check_clear`) that returns order dicts; the ROS wrapper publishes them on `/maze_team/decision` with a `type` field. `MazeSolver` replaces its local head-on yield (team mode) by a `HEAD_ON` report plus a 2 s wait, and runs the dormant states `YIELD_TURN` → `YIELD_ESCAPE` → `YIELD_WAIT` plus a new `HOLDING` state. Route following is a separate helper reused later by C2/C3.

**Tech Stack:** ROS 2 Humble rclpy, unittest/pytest, Gazebo Fortress.

**Spec:** `docs/superpowers/specs/2026-10-30-c4-head-on-hide-design.md`

## Global Constraints
- Values: `HEAD_ON_DECISION_SECONDS = 2.0`, `HIDE_DEPTH_M = 0.25`, `HOLD_TIMEOUT_SECONDS = 20.0`, clear distance 0.3 m, branch-occupied radius 0.5 m, distance tie 0.05 m.
- Hide branch order: BLOCKED or visited-and-unclaimed → OPEN unvisited → claimed.
- A robot already escaping/hiding is never ordered to yield again.
- Solo mode, yellow/green modes and the no-coordinator fallback keep today's behaviour.
- No speed or gain tuning; new sim-vs-real findings go to `toRealbot.md`.

## Review Focus
1. Both robots of a pair send HEAD_ON within one tick: exactly one YIELD and one HOLD — Task 1 `test_duplicate_head_on_gives_no_new_orders`.
2. Yielder whose trail node P itself has a free branch: route is the single step at P, no detour — Task 1 `test_hide_at_the_node_just_left`.
3. CLEAR must not fire while the holder is still in the corridor before H — Task 1 `test_no_clear_before_holder_passes`.
4. A HOLD that never gets GO must not freeze the robot — Task 2 `test_hold_times_out`.
5. After CLEAR the robot's request at H must not offer the hiding branch and must not link a bogus edge — Task 3 `test_clear_backs_out_and_asks_without_hide_branch` + Task 1 `test_hidden_sets_trail_to_hide_branch`.

---

### Task 1: Ledger encounters

**Files:**
- Modify: `mbot2_control/mbot2_control/team_coordinator.py`
- Create: `mbot2_control/test/test_team_hide.py`

**Interfaces:**
- Consumes: C1 nodes/branches/`trails`/`route()`.
- Produces:
  - `TeamLedger.head_on(robot: str, peer: str, now: float) -> list[dict]`: `[]` if the pair already has an open encounter, if either robot is the yielder of an open encounter (then the other one gets the YIELD if it has an option, else `[]`), or if neither has an option.
  - `TeamLedger._hide_option(robot: str, other: str) -> dict | None` → `{'distance', 'route', 'hide_node', 'hide_dir', 'hide_heading'}`; `route` steps as in `route()`, last step `{'node': H, 'exit_dir': hide_dir, 'heading': hide_heading}`.
  - `TeamLedger.hidden(robot: str) -> list[dict]` (GO for the holder; sets branch `hidden_by`, trail `(H, hide_dir)`).
  - `TeamLedger.check_clear(now: float) -> list[dict]` (CLEAR when the holder's trail differs from the one recorded at GO and it is > 0.3 m from H, or the holder timed out; removes `hidden_by`, closes the encounter).
  - Orders: `{'type': 'YIELD'|'HOLD'|'GO'|'CLEAR', 'robot', 'encounter': int, ...}`; YIELD adds `route, hide_node, hide_dir, hide_heading`.
  - `request_branch` skips branches whose `hidden_by` is another robot.
  - ROS wrapper: `HEAD_ON` → publish `head_on` orders; `HIDDEN` → publish `hidden` orders; after every `POSE` publish `check_clear(now)`.

- [ ] **Step 1: Write the failing tests** in `test/test_team_hide.py` on a hand-built map (helper joins nodes like `test_team_map.square`), robots placed with `update_pose` and `trails` set directly:
  - `test_closer_robot_hides`: mbot1 0.2 m from its P with a free branch, mbot2 0.6 m from its P → orders: YIELD to mbot1, HOLD to mbot2.
  - `test_equal_distance_uses_route_order`
  - `test_only_robot_with_option_hides`
  - `test_no_option_no_orders` → `[]`
  - `test_hide_branch_order`: H has a BLOCKED branch and an OPEN unvisited one → hide_dir is the BLOCKED one; the arrival branch and a branch with a robot 0.3 m along it are never chosen.
  - `test_hide_at_the_node_just_left`: route has one step, node == P.
  - `test_duplicate_head_on_gives_no_new_orders`
  - `test_hidden_sends_go_and_marks_branch`; `test_hidden_sets_trail_to_hide_branch`
  - `test_no_clear_before_holder_passes`; `test_clear_after_holder_passes` (holder trail changes and holder 0.4 m from H → CLEAR, `hidden_by` removed)
  - `test_request_branch_skips_hidden_branch`
  - `test_escaping_robot_is_not_asked_to_yield_again`
- [ ] **Step 2: Run** `cd ~/ros2_ws/src/mbot2_control && python3 -m pytest -q test/test_team_hide.py` — Expected: FAIL (`head_on` missing).
- [ ] **Step 3: Implement** in `team_coordinator.py` (Dijkstra for `_hide_option` may reuse a private helper shared with `route()`).
- [ ] **Step 4: Run** `python3 -m pytest -q test/test_team_hide.py test/test_team_map.py test/test_team_coordinator.py` — Expected: only the baseline `test_stale_claim_can_be_reused` fails.

### Task 2: Robot reports HEAD_ON, waits, holds

**Files:**
- Modify: `mbot2_control/mbot2_control/maze_solver.py`
- Test: `mbot2_control/test/test_junction_scan.py`

**Interfaces:**
- Produces: constants above; state `HOLDING`; `self.head_on_wait = {'peer', 'since'} | None`; `_on_team_decision` dispatches messages with `type` addressed to this robot to `_on_team_order(order)` (decisions without `type` keep the old path); `self.team_encounter` (dict of the current order).
- Behaviour: team mode head-on detection → zero cmd, send `{'type': 'HEAD_ON', 'peer'}` once, wait; YIELD → Task 3 entry `_begin_escape(order)`; HOLD → `HOLDING` (remember resume state); GO or `HOLD_TIMEOUT_SECONDS` → back to the remembered state; no order within `HEAD_ON_DECISION_SECONDS` → old `_begin_yield(peer)` when the peer has priority, else continue. Head-on and wall checks skip `YIELD_TURN/YIELD_ESCAPE/YIELD_WAIT/HOLDING` (ultrasonic in `YIELD_ESCAPE` only stops, never reports a wall).

- [ ] **Step 1: Write the failing tests**: `test_head_on_reports_and_stops` (peer RETURNING ahead → one `HEAD_ON`, linear 0, not `YIELDING`), `test_no_answer_falls_back_to_old_yield` (clock + 2.1 s → `YIELDING`), `test_hold_then_go_resumes`, `test_hold_times_out`.
- [ ] **Step 2: Run** `python3 -m pytest -q test/test_junction_scan.py` — Expected: the 4 new tests FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `python3 -m pytest -q test/` — Expected: only the 3 baseline failures; earlier yield tests that assumed an immediate local yield in team mode are updated to the new flow, each such change ledgered as a Ruling.

### Task 3: Escape along the route, hide, resume

**Files:**
- Modify: `mbot2_control/mbot2_control/maze_solver.py`
- Test: `mbot2_control/test/test_junction_scan.py`

**Interfaces:**
- Consumes: Task 2 `_on_team_order`; C1 `_known_node`, `_start_junction_scan`, `_finish_junction_scan`.
- Produces: `_begin_escape(order)`; `_follow_route_step(node: dict) -> bool` (if `node['id']` is the next route step: start a turn to the step heading, pop the step, return True); states `YIELD_TURN` (180° pivot), `YIELD_ESCAPE` (line following like FOLLOW, outer hits latched, at a hit the known node goes through `_follow_route_step` instead of request/scan/NODE), hiding leg (after the last step: follow the hide branch until the axle is `HIDE_DEPTH_M` from H, then stop, send `HIDDEN`, `YIELD_WAIT`); CLEAR in `YIELD_WAIT` → reverse straight until the axle is within `CENTER_TOLERANCE_M` of H, trim `self.junctions` to the entry within 0.25 m of H (append H if none), then `_request_team_branch(False, ..., approach_heading=hide_heading + pi, exit_headings=<team branches of H except hide_dir>)` and `JUNCTION_PAUSE`.

- [ ] **Step 1: Write the failing tests** (FloorHarness, team map in `team_nodes`): floor = corridor east–west with a corner at its west end leading north to junction H that has a free east branch; robot facing east in the corridor gets YIELD with route [corner → H, H → hide]:
  - `test_yield_turns_round_follows_route_and_hides`: ends in `YIELD_WAIT`, one `HIDDEN`, axle 0.25 ± 0.03 m from H along the hide branch, no `BRANCH_REQUEST`/`NODE` sent during the escape.
  - `test_clear_backs_out_and_asks_without_hide_branch`: then CLEAR → `JUNCTION_PAUSE`, last `BRANCH_REQUEST` has no choice heading within 0.3 rad of `hide_heading`, axle within 0.03 m of H.
- [ ] **Step 2: Run** `python3 -m pytest -q test/test_junction_scan.py` — Expected: the 2 new tests FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `python3 -m pytest -q test/` — Expected: only the 3 baseline failures; lint counts not above 10 / 22.

### Task 4: Docs and sim verification

**Files:** `mbot2_control/MULTI_ROBOT.md`, `toRealbot.md`

- [ ] **Step 1:** Update both docs (messages, flow; world-frame positions and wall-clock timers in `toRealbot.md`).
- [ ] **Step 2:** Announce to the user, check no other Gazebo, rebuild with `--symlink-install`, assert installed sources contain `_begin_escape` and `head_on`.
- [ ] **Step 3:** 600 s 3-robot sim with `/maze_team/state` recorded. Expected: no pair frozen face-to-face > 30 s, all 3 `GOAL_REACHED`, every YIELD followed by HIDDEN and CLEAR; report counts and durations.
- [ ] **Step 4:** Final whole-branch review, fix Critical/Important (TDD), ask the user before committing.
