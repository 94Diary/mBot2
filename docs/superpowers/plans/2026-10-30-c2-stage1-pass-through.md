# C2 Stage 1 Pass-through Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A FOLLOW robot asks the team about a known junction up to 0.40 m before it, drives straight through without stopping when told straight, and turns without scan or pause when told to turn.

**Architecture:** `MazeSolver` keeps `self.ahead_request` (node + request id). FOLLOW looks for a known junction ahead and sends the request; the black-mode outer-hit handler checks it first: straight → apply choice, record the junction, keep following; turn → axle centring then `_leave_junction_center`; no answer yet → centring then `JUNCTION_PAUSE` on the same request. Turn-back, yield and line loss cancel it.

**Spec:** `docs/superpowers/specs/2026-10-30-c2-pass-through-and-dispatch-design.md` (stage 1)

## Global Constraints
- `LOOKAHEAD_M = 0.40`, along ≥ 0.05, lateral ≤ 0.05, map fresher than `TEAM_MAP_FRESH_SECONDS`; one request per node; FOLLOW + black mode + team only.
- No driving speed changes.

## Review Focus
1. One request per junction even when the answer is late — `test_waits_when_answer_late_without_second_request`.
2. A pending ahead request is cancelled on turn-back / yield / line loss — `test_turn_back_cancels_ahead_request`.
3. Passing straight must not re-trigger on the same junction — `test_passes_straight_without_stopping` (no CENTERING after the pass).
4. The robot's own junction stack records the passed junction — `test_passes_straight_without_stopping` asserts `junctions[-1]` world position.
5. A stale ahead request for a node already behind is dropped — `test_stale_ahead_request_dropped`.

### Task 1: Ask ahead, pass, turn, wait, cancel
**Files:** `mbot2_control/mbot2_control/maze_solver.py`, `mbot2_control/test/test_junction_scan.py`
- [ ] Tests: `test_asks_ahead_for_known_junction`, `test_passes_straight_without_stopping`, `test_turns_at_known_junction_without_scan_or_wait`, `test_waits_when_answer_late_without_second_request`, `test_turn_back_cancels_ahead_request`, `test_stale_ahead_request_dropped`. Run → FAIL; implement; suite → only baseline fails; lint not above 9/22.

### Task 2: Docs, sim, review
- [ ] `MULTI_ROBOT.md`, `toRealbot.md`.
- [ ] Announce; rebuild; assert source has `ahead_request`; 600 s sim, timestamped state. Expected: ≥1 `ผ่าน J... ตรงไปโดยไม่หยุด`, all 3 parked, no freeze/reverse; report goal time.
- [ ] Fresh review; fix Critical/Important; ask before committing.
