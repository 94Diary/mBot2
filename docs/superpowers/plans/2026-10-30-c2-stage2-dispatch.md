# C2 Stage 2 Team Assignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Before the goal is known, the team assigns each idle robot the nearest unexplored branch (closest pairs first, reassign when another idle robot is ≥ 0.5 m closer) and answers its junction requests along the way to that branch.

**Spec:** `docs/superpowers/specs/2026-10-30-c2-pass-through-and-dispatch-design.md` (stage 2)

## Global Constraints
- Reassignment margin 0.5 m; assignments cleared once the goal is known; robot behaviour unchanged except reason labels.

## Review Focus
1. A busy robot never gets an assignment — `test_busy_robot_not_assigned`.
2. Two idle robots never share a task — `test_two_idle_robots_get_different_tasks`.
3. No ping-pong between two robots — `test_no_reassignment_below_margin`.
4. Assignment cleared when the robot enters its branch — `test_assigned_branch_given_at_task_junction`.
5. Falls back to normal ranking when the task is unreachable on the map — `test_unreachable_task_uses_normal_ranking`.

### Task 1: Ledger assignment + steering
**Files:** `team_coordinator.py`, `maze_solver.py` (labels), create `test/test_team_dispatch.py`
- [ ] Tests (spec table + Review Focus). Run → FAIL; implement `_is_task`, `_busy`, `_robot_reach`, `_update_assignments`, request steering, snapshot; suite → only baseline fails.

### Task 2: Docs, sim, review
- [ ] `MULTI_ROBOT.md`; announce; rebuild; assert `_update_assignments`; 600 s sim. Expected: all 3 parked, ≥1 `สำรวจกิ่งที่ทีมจองให้`, no freeze/reverse.
- [ ] Fresh review; fix Critical/Important; ask before committing.
