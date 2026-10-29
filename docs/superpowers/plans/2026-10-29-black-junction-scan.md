# Black Junction Scan Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** With `junction_color: 'black'`, robots find junctions on an all-black maze by stopping on an outer-sensor hit, centring the wheels and sweeping in place.

**Architecture:** `QuadRGBArray` latches outer-sensor hits between ticks and reports no junction colour in black mode. `MazeSolver` gains one state, `JUNCTION_SCAN`, entered from `CENTERING`; the scan result (exit angles) decides dead end / false trigger / corner / junction. Junction nodes store measured world headings, which go to the team as choice headings, so `_apply_team_choice` already turns to the measured exit.

**Tech Stack:** ROS 2 Humble rclpy, unittest/pytest, Gazebo Fortress.

**Spec:** `docs/superpowers/specs/2026-10-29-black-junction-scan-design.md`

## Global Constraints
- Yellow/green junction modes behave exactly as before; `test_line_recovery.py` stays green.
- No speed or gain tuning. New constants only for scan geometry: `SCAN_LIMIT_RAD = radians(130)`, straight window ≤ 40°, side window 50°–130°.
- Real-robot sensors only.

## Review Focus
1. A trigger right after leaving a junction/corner must not start a second scan (escape distance `JUNCTION_ESCAPE_DISTANCE_M`).
2. Yield during `JUNCTION_SCAN` must leave no stale scan state.
3. Returning robot reaching its saved junction must not rescan; it uses stored headings.
4. A scan on the inbound line behind (θ≈180°) must never count as an exit.
5. Solo mode with several exits must pick by `junction_choice` and turn to the measured heading.

---

### Task 1: Sensor latch and black mode
**Files:** Modify `mbot2_control/modules/sensors/quad_rgb_sensor.py`; Test `test/test_junction_scan.py`
**Produces:** `QuadRGBArray.on_outer_hit` (callable or None, called from the image callback when an outer sensor is black while an inner sensor is black); `junction_color == 'black'` → `yellow` all False.
- [ ] Test `test_black_mode_reports_no_junction_colour` and `test_outer_hit_callback_fires_between_ticks` (drive callbacks with fake Image msgs). Watch fail. Implement. Pass.

### Task 2: Outward scan
**Files:** Modify `maze_solver.py`; Test `test/test_junction_scan.py` (FakeFloor harness: segments → per-sensor black from pose)
**Consumes:** Task 1 hook. **Produces:** state `JUNCTION_SCAN`; `self.black_mode`; node fields `exit_headings` (dict name → world heading).
- Trigger in `FOLLOW` (black mode, `distance_total >= self.trigger_resume_distance`): latched hit → record hit position → `CENTERING` (wheels to the hit point; reverse at `SEARCH_SPEED` if passed by > 0.02 m) → `JUNCTION_SCAN` (+130° then −130°, record inner-black yaws into windows).
- Classify: 0 exits → BLOCKED (if active branch) + U-turn to h0+π → RETURNING; straight only → turn to it, FOLLOW; one side → turn to it, FOLLOW (no record, no request); ≥ 2 → record node (`left_open`, `right_open`, `straight_open`, `exit_headings`), team request with measured headings / solo choice, then existing JUNCTION_PAUSE / leave path.
- Tests (spec table): T junction, corner, false trigger, dead end, dead end with BLOCKED, 5 cm overshoot, rotation ≤ 400°, team choices only scanned exits, yield during scan clears scan.
- [ ] Write tests, watch fail, implement, pass; run whole `test_line_recovery.py` + `test_team_coordinator.py`.

### Task 3: Returning
**Files:** Modify `maze_solver.py`; Test `test/test_junction_scan.py`
- Saved-junction arrival unchanged (`_return_reached_junction`); returning request uses `exit_headings` when the node has them.
- Black-mode trigger while RETURNING away from the saved junction → scan; corner → turn and stay RETURNING; ≥ 2 exits → record as missed junction and request as returning.
- Tests: `test_return_request_uses_measured_headings`, `test_corner_while_returning_keeps_returning`.
- [ ] TDD as above.

### Task 4: Map, launch, sim
**Files:** `mbot2_seed7_maze.sdf` (green `0.0 0.8 0.2` → `0.02 0.02 0.02`), `two_mbot2.launch.py` (`junction_color: 'black'`), `src/CLAUDE.md` colour note.
- [ ] Build; 3-robot headless sim 6 min; count `สแกนแยก`, corners, false triggers, dead-end U-turns, missed junctions, goal. Old-map single-robot sim reaches goal. Report; do not tune.
