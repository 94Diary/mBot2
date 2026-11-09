# Seed7 Map Junction Reading — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** ทำให้หุ่นอ่าน `mbot2_seed7_maze` ได้ถูกต้อง ได้แก่ แยก กิ่งข้าง ทางตรง และทางตัน โดยใช้ตรรกะเลือกกิ่งของทีมแบบเดิม

**Architecture:** แก้ "ตา" ใน `maze_solver.py` และ `quad_rgb_sensor.py` (กฎ 1, 2, 5) เพิ่มพารามิเตอร์รัศมีให้ `team_coordinator.py` (กฎ 3) และเพิ่มกำแพงที่ทางตันในไฟล์ world (กฎ 4) ไม่แตะ `TeamLedger.request_branch`

**Tech Stack:** ROS 2 Humble, rclpy, Gazebo Fortress, pytest

**Spec:** `docs/superpowers/specs/2026-10-29-seed7-map-junction-design.md`

## Global Constraints
- ทำบน branch `fix/mbot2-collective-decision-making`
- รันเทสต์: `cd ~/ros2_ws/src/mbot2_control && source /opt/ros/humble/setup.bash && python3 -m pytest test -q`
- baseline ที่ล้มอยู่ก่อนแล้วและต้องปล่อยไว้: `test_flake8` (11 ข้อ), `test_pep257`, `test_stale_claim_can_be_reused` ห้ามเพิ่มข้อ lint ใหม่
- ค่าเริ่มต้นของ `junction_match_radius` = 1.00, ค่าใน `two_mbot2.launch.py` = 0.25
- ระยะเพลาล้อจาก `base_link`: `WHEEL_X_OFFSET_M = -0.015` (ตรงกับ `wheel_x` ใน URDF)
- ข้อความ log ในโค้ดเป็นภาษาไทยตามสไตล์เดิม
- ต้องได้รับอนุญาตจากผู้ใช้ก่อน commit ทุกครั้ง

## Review Focus
1. **แผนที่เก่ายังทำงานได้** (`simple_branch_maze` + `gz_sim.launch.py`): หุ่นยังต้องเห็นแยก เดินตรงไปชนกล่องกั้น แล้วย้อนกลับได้ → ใช้ซิมใน Task 5 Step 4
2. **หุ่นหลีกทางระหว่างอยู่ที่แยก**: ถ้าหุ่น yield ตอน `CENTERING` (ก่อนส่งคำถามให้ทีม) จะต้องไม่ส่ง `CANCEL_BRANCH` ที่ไม่มีอยู่จริง และต้องไม่ค้าง → เทสต์ใน Task 3
3. **ขากลับผ่านแยกที่ไม่มีทางตรง**: ตอนย้อนกลับมาที่แยกนั้น ทีมต้องไม่ได้รับตัวเลือกทางตรง → เทสต์ใน Task 3
4. **โหมดไม่ใช้ทีม** (`team_enabled:=false`): หุ่นต้องไม่เลือกทางตรงเมื่อทางตรงปิด → เทสต์ใน Task 3
5. **เส้นหายกลางทางทั้งที่ไม่ใช่ทางตัน**: ต้องส่ง `RETRY_BRANCH` ไม่ใช่ `BLOCKED` → เทสต์ใน Task 4

---

### Task 1: รัศมีจับคู่แยกของตัวกลาง (กฎ 3)

**Files:**
- Modify: `mbot2_control/mbot2_control/team_coordinator.py` (`TeamLedger.__init__`, `_junction`, `TeamCoordinator.__init__`)
- Modify: `mbot2_description/launch/two_mbot2.launch.py` (Node `team_coordinator`)
- Modify: `mbot2_control/MULTI_ROBOT.md` (บรรทัด "การระบุ “แยกเดียวกัน” ใช้ระยะไม่เกิน 1.00 เมตร")
- Test: `mbot2_control/test/test_team_coordinator.py`

**Interfaces:**
- Produces: `TeamLedger(match_radius: float = MATCH_RADIUS_M)`, ROS param `junction_match_radius` (float, default 1.00)

- [ ] **Step 1: เขียนเทสต์ที่ต้องล้ม** `test_grid_map_keeps_neighbouring_junctions_apart`: สร้าง `TeamLedger(match_radius=0.25)` ส่ง `request_branch` ที่ (−1.25, −1.25) แล้วที่ (−0.75, −1.25) จากนั้น assert `first['junction_id'] != second['junction_id']` และ `second['junction_x'] == -0.75`
- [ ] **Step 2: รันให้เห็นว่าล้ม** `python3 -m pytest test/test_team_coordinator.py -k grid` → TypeError unexpected keyword `match_radius`
- [ ] **Step 3: แก้โค้ด** เก็บ `self.match_radius` ใน ledger แล้วให้ `_junction` ใช้ค่านี้ ให้ node `declare_parameter('junction_match_radius', MATCH_RADIUS_M)` แล้วส่งต่อให้ ledger ใน launch เพิ่ม `parameters=[{'junction_match_radius': 0.25}]` พร้อมคอมเมนต์ว่าแยกใน seed7 อยู่บนตาราง 0.5 ม. และแก้บรรทัดใน MULTI_ROBOT.md ให้บอกทั้งค่าเริ่มต้นและค่าของ seed7
- [ ] **Step 4: รันเทสต์ให้ผ่าน** `python3 -m pytest test/test_team_coordinator.py test/test_team_ros.py` → ล้มแค่ `test_stale_claim_can_be_reused`
- [ ] **Step 5: commit** (ขออนุญาตก่อน) `feat(coordinator): configurable junction match radius for grid maps`

### Task 2: กิ่งข้างจากสีแยก (กฎ 1) และ harness เทสต์

**Files:**
- Modify: `mbot2_control/mbot2_control/modules/sensors/quad_rgb_sensor.py` (`junction_sides`)
- Create: `mbot2_control/test/test_line_recovery.py`

**Interfaces:**
- Produces: คลาส `SolverHarness(node)` ใน `test_line_recovery.py` มีเมธอด `set_colours(black=(F,)*4, junction=(F,)*4)` และ `tick(dt=0.1) -> Twist` ซึ่งอัปเดตตำแหน่งผ่าน `node._update_pose(...)` ทำให้ `last_odom_at`, `last_seen_at`, `last_team_state_at` สดทุก tick และ stub `cmd_pub.publish` ให้เก็บลงลิสต์ `sent`; คลาส `LineRecoveryTest(unittest.TestCase)` ทำ `rclpy.init()/shutdown()` ระดับ class และใน `setUp` สร้าง `MazeSolver()` แล้วเรียก `_update_pose(0, 0, 0)`
- Produces: `junction_sides() -> tuple[bool, bool]` = (ตัวนอกซ้ายเห็นดำหรือสีแยก, ตัวนอกขวาเห็นดำหรือสีแยก)

- [ ] **Step 1: เขียนเทสต์ที่ต้องล้ม** `test_outer_sensor_on_junction_colour_confirms_side_branch`: อยู่ในสถานะ `APPROACH_JUNCTION` และ `junction=(True, True, True, False)` แล้ว tick 3 ครั้ง ต้องได้ `len(node.junctions) == 1`, `node.junctions[-1]['left_open'] is True` และ `right_open is False` (ไม่ assert ชื่อ state ถัดไป เพราะ Task 3 จะเปลี่ยนมัน)
- [ ] **Step 2: รันให้เห็นว่าล้ม** → `IndexError` หรือ assert ล้ม เพราะไม่มีแยกถูกบันทึก
- [ ] **Step 3: แก้โค้ด** ให้ `junction_sides` คืนค่า `(black[0] or yellow[0], black[3] or yellow[3])` และเพิ่มบรรทัดใน docstring ว่าแผนที่ใหม่ทาสีแยกทับกิ่งข้าง
- [ ] **Step 4: รันเทสต์ให้ผ่าน** `python3 -m pytest test/test_line_recovery.py`
- [ ] **Step 5: commit** (ขออนุญาตก่อน) `fix(sensors): outer sensor on junction colour marks a side branch`

### Task 3: เข้ากลางแยกก่อน แล้วค่อยถามทีมว่ามีทางตรงไหม (กฎ 2)

**Files:**
- Modify: `mbot2_control/mbot2_control/maze_solver.py`: `_request_team_branch` (~468), `_at_junction` (~765), สถานะ `CENTERING` (~1039), สถานะ `JUNCTION_PAUSE` (~1374)
- Test: `mbot2_control/test/test_line_recovery.py`

**Interfaces:**
- Consumes: `SolverHarness` จาก Task 2
- Produces:
  - `_request_team_branch(returning, left_open, right_open, position_override=None, straight_open=True)`: ตอนขาไปจะใส่ `straight` เฉพาะเมื่อ `straight_open` เป็นจริง ตอนขากลับจะใส่ `(original, False)` เฉพาะเมื่อ `node.get('straight_open', True)` เป็นจริง
  - field ใหม่ในแต่ละแยก: `self.junctions[i]['straight_open']: bool`
  - flag ใหม่: `self.junction_needs_request: bool` (เป็น True ระหว่างขาไปที่กำลังเข้ากลางแยกแต่ยังไม่ได้ถามทีม)
  - helper ใหม่: `_leave_junction_center()` ย้ายมาจาก CENTERING ส่วน "centered" เดิม: ถ้า `branch_angle == 0.0` ให้ข้ามแยกไป `BRANCH_CAPTURE` ถ้าไม่ใช่ให้เรียก `_start_turn(branch_angle, FOLLOW, 'BRANCH')`

ลำดับใหม่ตอนขาไป: `_at_junction(returning=False)` บันทึกแยก (`straight_open` เริ่มเป็น True) ตั้ง `active_junction_world = world_position` ของแยกนี้ และ `center_start/center_heading` ตั้ง `junction_needs_request = True` แล้ว**ไป `CENTERING` โดยยังไม่ส่งคำถาม** → เมื่อเข้ากลางแยกแล้วและ `junction_needs_request`: ตั้ง `straight_open = any(black) or any(yellow)` เก็บลงแยก ถ้าใช้ทีม ส่ง `_request_team_branch(False, left, right, straight_open=...)` แล้วไป `JUNCTION_PAUSE` ถ้าไม่ใช้ทีมและ `branch_angle == 0` แต่ทางตรงปิด ให้เลือกกิ่งข้างที่เปิด แล้วเรียก `_leave_junction_center()` → ใน `JUNCTION_PAUSE` เมื่อ `_apply_team_choice()` สำเร็จ ถ้าเป็นขาไปให้เรียก `_leave_junction_center()` ทันที ถ้าเป็นขากลับให้ไป `CENTERING` ตามเดิม

- [ ] **Step 1: เขียนเทสต์ที่ต้องล้ม** (team on, stub `_team_report` ให้เก็บลงลิสต์ `reports`)
  - `test_junction_request_waits_until_wheels_reach_centre`: หลังยืนยันแยก จะต้องยังไม่มี `BRANCH_REQUEST` ใน `reports` และ state == `CENTERING`
  - `test_no_straight_offered_when_floor_ahead_is_blank`: tick ใน CENTERING จนเข้ากลางแยกโดยเซนเซอร์ว่างหมด คำถามสุดท้ายต้องไม่มี `straight` ใน `choices` และ `junctions[-1]['straight_open'] is False`
  - `test_straight_offered_when_junction_colour_ahead`: เหมือนข้างบนแต่ `junction=(False, True, True, False)` ต้องมี `straight`
  - `test_return_trip_omits_missing_straight`: ตั้ง `junctions=[{..., 'straight_open': False, 'left_open': True, 'right_open': False}]` แล้วเรียก `_request_team_branch(True, ...)` ชื่อ choices ต้องไม่มีทิศ `original` ที่ fallback=False
  - `test_yield_during_centering_sends_no_cancel`: อยู่ใน `CENTERING` และ `junction_needs_request=True` แล้วเรียก `_begin_yield('mbot0')` (มี peer ใน `team_peers`) ต้องไม่มี `CANCEL_BRANCH` ใน reports และ `junction_needs_request` ต้องกลายเป็น False
  - `test_solo_robot_turns_when_straight_closed` (team off, `junction_choice='straight_first'` ตั้งผ่าน `node.junction_choice`): เข้ากลางแยกโดยพื้นข้างหน้าว่าง และ `left_open=True` ต้องได้ state == `TURNING` และ `branch_angle > 0`
- [ ] **Step 2: รันให้เห็นว่าล้ม** → ล้มทั้ง 6 ตัว เพราะคำถามถูกส่งตั้งแต่ก่อนเข้ากลางแยก และไม่มีฟิลด์ `straight_open`
- [ ] **Step 3: แก้โค้ด** ตามลำดับใหม่ข้างบน และใน `_begin_yield` ให้ตั้ง `junction_needs_request = False` ด้วย (หลัง yield หุ่นจะกลับไปที่ `FOLLOW` หรือ `RETURNING` แล้วเห็นแยกใหม่เอง)
- [ ] **Step 4: รันเทสต์ให้ผ่าน** ทั้งไฟล์ แล้วรันทั้งชุด → baseline เท่าเดิม
- [ ] **Step 5: commit** (ขออนุญาตก่อน) `fix(solver): centre on junction and read straight branch before asking team`

### Task 4: ตาข่ายนิรภัยของโหมดเส้นหาย (กฎ 5)

**Files:**
- Modify: `mbot2_control/mbot2_control/maze_solver.py`: ค่าคงที่หัวไฟล์, `__init__`, `_begin_line_recovery` (~696), `_recover_lost_line` (~713)
- Test: `mbot2_control/test/test_line_recovery.py`

**Interfaces:**
- Produces: ค่าคงที่ `WHEEL_X_OFFSET_M = -0.015`, helper `_axle_point(pose) -> (x, y)`, state ใหม่ `self.scan_rotation: float`, `self.scan_last_yaw: float`

ระยะถึง anchor ใน `_recover_lost_line` ให้วัดระหว่าง `_axle_point(self.pose)` กับ `_axle_point(anchor)` ค่า `scan_rotation` รีเซ็ตใน `_begin_line_recovery` เฉพาะเมื่อ anchor ใหม่อยู่ห่างจาก anchor เดิม > 0.05 ม. หรือยังไม่มี anchor ส่วน `scan_last_yaw` ตั้งใหม่ทุกครั้งที่เข้า `SCAN_AT_ANCHOR` เมื่อ `scan_rotation >= 2π` ให้ทำ: log WARN, ส่ง `RETRY_BRANCH` ของ `active_team_branch` (ถ้ามี) แล้วตั้งเป็น None, ตั้ง `line_lost_anchor = None` และเรียก `_start_turn(math.pi, RETURNING, 'UTURN')`

- [ ] **Step 1: เขียนเทสต์ที่ต้องล้ม** ให้ harness หมุนรอบ**เพลาล้อ** คือตำแหน่ง `base_link` ต้องเคลื่อนเป็นวงกลมรัศมี 0.015 ม. ไม่ใช่หมุนอยู่กับที่
  - `test_scan_finishes_within_one_revolution`: เซนเซอร์ว่างหมด เมื่อจบ state == `TURNING`, `turn_kind == 'UTURN'` และมุมที่หมุนรวม < 2π + 0.5
  - `test_scan_count_survives_drifting_off_the_anchor`: ระหว่าง SCAN เลื่อนหุ่นออก 0.03 ม. ทุก 40 tick ต้องยังจบที่ `UTURN` ภายใน 1500 tick
  - `test_lost_line_reports_retry_not_blocked`: มี `active_team_branch` อยู่ ต้องมี `{'type': 'RETRY_BRANCH', **branch}` ใน reports ไม่มี `BLOCKED` และ `active_team_branch is None`
- [ ] **Step 2: รันให้เห็นว่าล้ม** → state ค้างที่ `LINE_LOST`
- [ ] **Step 3: แก้โค้ด** ตาม Interfaces ข้างบน
- [ ] **Step 4: รันเทสต์ให้ผ่าน** ทั้งไฟล์ และยืนยันว่าเทสต์จับบั๊กได้จริง: ใส่ `self.scan_rotation = 0.0` กลับไปตอนเข้า SCAN ชั่วคราว เทสต์การเลื่อนต้องล้ม แล้วเอาออก
- [ ] **Step 5: commit** (ขออนุญาตก่อน) `fix(solver): bound line-loss scan to one revolution around the wheel axle`

### Task 5: กำแพงทางตัน (กฎ 4) และทดสอบในซิม

**Files:**
- Modify: `mbot2_description/worlds/mbot2_seed7_maze.sdf` (เพิ่ม model ใหม่ก่อน `</world>`)

- [ ] **Step 1: เพิ่ม model `dead_end_walls`** (`<static>true</static>`, 1 link, แต่ละกำแพงมีทั้ง `<collision>` และ `<visual>`, สีเทา) ตามตำแหน่งที่กำหนดใน spec: (1.33, −1.25) ขนาด 0.02×0.20×0.10, (1.33, 1.25) ขนาด 0.02×0.20×0.10, (−0.25, −0.17) ขนาด 0.20×0.02×0.10 กึ่งกลางแนวตั้ง z = 0.05
- [ ] **Step 2: build** `cd ~/ros2_ws && colcon build --packages-select mbot2_control mbot2_description --symlink-install` → `2 packages finished`
- [ ] **Step 3: ซิมแผนที่ใหม่** `LIBGL_ALWAYS_SOFTWARE=1 timeout -s INT 360 ros2 launch mbot2_description two_mbot2.launch.py headless:=true robot_count:=3 > sim.log` แล้วเช็ค log:
  - ทุกครั้งที่มี `เห็นแยก` ต้องตามด้วย `เลือก...ที่ J<n>`
  - มี `เจอกำแพง` อย่างน้อย 1 ครั้ง และตามด้วย `ย้อนกลับไปเลือกทางที่แยก`
  - ไม่มีหุ่นที่ค้างใน `LINE_LOST` เกิน 1 รอบโดยไม่มีข้อความ "หมุนหาเส้นครบรอบ"
  - มี `Goal reported by` จาก coordinator
- [ ] **Step 4: ซิมแผนที่เก่า** `gz_sim.launch.py` (headless ไม่ได้ ใช้ `timeout` และ `LIBGL_ALWAYS_SOFTWARE=1`) + `ros2 run mbot2_control maze_solver` เป็นเวลา 3 นาที → ต้องมี `เห็นแยก` และ `เจอกำแพง`
- [ ] **Step 5: รันเทสต์ทั้งชุด** → baseline เท่าเดิม และไม่มี lint ใหม่
- [ ] **Step 6: commit** (ขออนุญาตก่อน) `feat(world): walls at seed7 dead ends`

ถ้า Step 3 ไม่ผ่านเพราะหุ่นหลุดเส้นบ่อย (ความเสี่ยงที่ระบุไว้ใน spec) ให้หยุดและรายงานผลซิม ห้ามจูนความเร็วหรือ gain เอง
