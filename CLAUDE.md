# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# mBot2 Swarm Maze Exploration

## เป้าหมายโปรเจกต์
ฝูงหุ่น mBot2 (2–3 ตัว) สำรวจเขาวงกตแบบเดินตามเส้นพร้อมกัน แบ่งกันไปคนละกิ่ง จำกิ่งที่เป็นทางตันร่วมกัน และเมื่อตัวใดเจอเป้าหมาย ตัวอื่นจะมุ่งไปรวมกันที่จุดนั้น

**ข้อจำกัดที่สำคัญที่สุดคือโค้ดต้องเอาไปรันบน mBot2 จริงได้** ไม่ใช่ใช้ได้แค่ในซิม
- ใช้ได้เฉพาะเซนเซอร์ที่ mBot2 จริงมี: อัลตราโซนิกจุดเดียว, IMU, Quad RGB 4 จุดใต้ท้อง, encoder ล้อ **ห้ามเพิ่ม LiDAR** หรือออกแบบให้ต้องพึ่ง SLAM ที่ใช้ LiDAR
- ห้ามโหลดแผนที่เขาวงกตไว้ล่วงหน้า ข้อมูลทางแยกต้องสร้างจากสิ่งที่หุ่นสังเกตเห็นเท่านั้น
- ในซิมตอนนี้ใช้ Gazebo ground truth (`use_ground_truth:=true`, topic `/world/<world>/dynamic_pose/info`) แทนการระบุตำแหน่ง เพราะ odom คลาดมากหลังเลี้ยว ส่วนนี้**ใช้กับหุ่นจริงไม่ได้** เวลาออกแบบต้องคิดไว้ว่าจะเอาอะไรมาแทน
- เป้าหมายในซิมคือหุ่นต้องถึงเส้นชัย ส่วนเรื่องที่ซิมต่างจากหุ่นจริงหรือต้องแก้ก่อนขึ้นหุ่นจริง **ให้จดลง `toRealbot.md` ทุกงาน** (แบ่งเป็น "ต้องรู้" และ "ต้องแก้แน่ๆ")

## Environment (ห้ามแนะนำอะไรที่ไม่ตรงกับนี้)
- Windows 11 + WSL2 Ubuntu 22.04, **ROS 2 Humble**
- **Gazebo Sim Fortress** (v6.x): ใช้คำสั่ง `ign gazebo` ไม่ใช่ `gazebo` หรือ `gz`; ชื่อ plugin เป็น `libignition-gazebo-*-system.so`; ข้อความใน bridge ใช้ `ignition.msgs.*`
- บน WSLg ต้องตั้ง `LIBGL_ALWAYS_SOFTWARE=1` (ใส่ไว้ใน `~/.bashrc` แล้ว) ถ้าไม่ตั้ง Gazebo จะ crash หรือ `ogre2` จะล้มในโหมด headless
- Git: https://github.com/94Diary/mBot2.git (repo อยู่ที่ `~/ros2_ws/src`) ทำงานเป็นทีมผ่าน feature branch + PR; GitHub Desktop ใช้กับ WSL ไม่ได้

## คำสั่ง
```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select mbot2_control mbot2_description --symlink-install
source install/setup.bash

# หลายหุ่น (ตัวหลัก): Gazebo + bridge + team_coordinator + maze_solver ทุกตัว
ros2 launch mbot2_description two_mbot2.launch.py            # robot_count:=2|3 (ค่าเริ่มต้น 3)
#   start_controllers:=false  เปิดแค่ซิม ไม่เปิดตัวควบคุม
#   headless:=true            ไม่เปิดหน้าต่าง 3D

# หุ่นเดี่ยว (โหมดเก่า, world = simple_branch_maze, ไม่มีทีม)
ros2 launch mbot2_description gz_sim.launch.py
ros2 run mbot2_control maze_solver

# ดูการตัดสินใจของทีม
ros2 topic echo /maze_team/decision   # หรือ /maze_team/state, /maze_team/report, /mbot1/trail
```

เทสต์ (อยู่ใน `mbot2_control/test/`):
```bash
colcon test --packages-select mbot2_control && colcon test-result --verbose
# รันเร็ว/รันตัวเดียว (ต้อง source /opt/ros/humble/setup.bash ก่อน)
cd ~/ros2_ws/src/mbot2_control && python3 -m pytest test/test_team_coordinator.py -k distinct
```
- `test_team_coordinator.py` ทดสอบ `TeamLedger` ล้วนๆ โดยไม่ต้องใช้ ROS; `test_team_ros.py` ทดสอบผ่าน topic จริงโดยไม่ต้องเปิด Gazebo; ที่เหลือคือ lint (flake8/pep257/copyright)
- ณ 2026-10-29 `test_stale_claim_can_be_reused` ล้มอยู่ก่อนแล้ว

เมื่อแก้ launch, xacro, `.sdf`, `package.xml` หรือ `setup.py` ต้อง build ใหม่และสั่ง launch ใหม่ แม้จะใช้ `--symlink-install` ก็ตาม

## สถาปัตยกรรม

### `mbot2_description/` (ament_cmake): โมเดล, world และ launch
- `urdf/mbot2.urdf.xacro` คือไฟล์เดียวที่นิยามหุ่นทั้งตัว รวมเซนเซอร์และ Gazebo plugin โดยรับ arg `robot_namespace` เพื่อใส่ prefix ให้ทุก Gazebo topic
- `launch/two_mbot2.launch.py`: ตำแหน่งเริ่มต้นของหุ่นอยู่ใน tuple `ROBOTS`; ส่งพารามิเตอร์ให้ `maze_solver` ของแต่ละตัว (`world_name`, `junction_color`, `goal_color`, spawn pose); controller แต่ละตัวเริ่มห่างกัน 8 วินาที; bridge สร้างจากฟังก์ชัน `bridge_arguments()` (`mbot1`/`mbot2` ใช้ bridge ตัวหนึ่ง ส่วน `mbot3` แยกอีกตัวตามเงื่อนไข)
- **ชื่อ world ถูกเขียนตายตัวไว้หลายที่** ถ้าเปลี่ยน world ต้องแก้ให้ครบ ได้แก่ `world_file`, topic `dynamic_pose/info` ใน `bridge_arguments` และพารามิเตอร์ `world_name` (ตอนนี้ launch หลายหุ่นใช้ `mbot2_seed7_maze`, หุ่นเดี่ยวใช้ `simple_branch_maze`)
- สีบนพื้นแต่ละ world ไม่เหมือนกัน: `simple_branch_maze` ใช้แยก=เหลือง เป้าหมาย=เขียว (ค่าเริ่มต้นของ node) ส่วน `mbot2_seed7_maze` แยกเป็นเส้นดำล้วน (`junction_color: 'black'`: หุ่นหยุดเมื่อตัวนอกเห็นเส้น แล้วหมุนสแกนหากิ่ง) เป้าหมาย=พื้นที่สีแดง 0.30×0.90 ม. ที่หุ่นทุกตัวต้องเข้าไปจอด (ตั้งผ่าน launch) เส้นทางเป็นสีดำ และกล่องเริ่มต้นเป็นสีน้ำเงิน

### `mbot2_control/` (ament_python): ตัวควบคุม
- `maze_solver.py` (~1400 บรรทัด) คือ node หนึ่งตัวต่อหุ่น ใช้ state machine ที่ `tick()` ทำงานที่ 10 Hz สถานะต่างๆ เป็นค่าคงที่แบบ string ที่หัวไฟล์ (`SEARCH`, `FOLLOW`, `APPROACH_JUNCTION`, `TURNING`, `RETURNING`, `YIELDING*`, `GOAL_REACHED` …) ค่าที่ใช้ปรับจูนเป็นค่าคงที่ตัวพิมพ์ใหญ่อยู่ด้านบนไฟล์ หุ่นเดินตามเส้นดำ ตรวจทางแยกด้วยสีแยก แล้วเก็บ breadcrumb stack ไว้ในเครื่องตัวเอง (`self.junctions`) เพื่อย้อนกลับเมื่อเจอทางตัน คล้ายแนวคิด Trémaux
- `team_coordinator.py` คือ node กลางตัวเดียว ตรรกะการตัดสินใจทั้งหมดอยู่ในคลาส `TeamLedger` ซึ่ง**ไม่มี import ของ ROS** เพื่อให้ unit test ได้ ส่วน `TeamCoordinator` เป็นแค่ ROS wrapper บางๆ
- `modules/sensors/` (`QuadRGBArray`, `UltrasonicSensor`) และ `modules/utils/encoder.py` เป็นตัวห่อ subscriber ที่ `maze_solver` สร้างขึ้นมาใช้
- เมื่อเพิ่ม node ใหม่ ต้องเพิ่ม entry point ใน `setup.py` → `console_scripts`

### โปรโตคอลของทีม (JSON ใน `std_msgs/String`)
- หุ่น → `/maze_team/report` มี field `type` เป็น `POSE` | `BRANCH_REQUEST` | `BLOCKED` | `RETRY_BRANCH` | `CANCEL_BRANCH` | `GOAL`
- coordinator → `/maze_team/decision` (กิ่งที่เลือกให้แต่ละ `request_id`) และ `/maze_team/state` (snapshot ทุก 0.5 วินาที: robots, junctions, goal) ทั้งสอง topic ใช้ QoS แบบ RELIABLE + TRANSIENT_LOCAL ฝั่ง subscriber ต้องตั้ง QoS ให้ตรงกัน
- ทางแยกแยกด้วยตำแหน่ง (`MATCH_RADIUS_M` = 1 m) และกิ่งแบ่งเป็น 4 ทิศ 90° (`direction_key`) ซึ่งเหมาะกับ maze ที่ทุกกิ่งตั้งฉากกัน
- ถ้า coordinator ไม่ทำงาน หุ่นจะหยุดรอที่ทางแยก ไม่เดาทางเอง หุ่นที่เจอกันบนเส้นแคบจะหลีกทางให้กันผ่าน state `YIELD_*` (ใช้ตำแหน่งเพื่อนจาก `/maze_team/state`)
- รายละเอียดพฤติกรรมและผลทดสอบดูที่ `mbot2_control/MULTI_ROBOT.md`

### Topics ของหุ่นแต่ละตัว (อยู่ใต้ `/<robot>/`)
`cmd_vel`, `odom`, `joint_states`, `tf`, `imu`, `ultrasonic/scan` (`LaserScan` ใช้แค่ `ranges[0]`), `quad_rgb_1..4/image` (กล้อง 1×1 พิกเซล), `trail`

## บั๊กที่เคยเจอและวิธีแก้ที่ใช้แล้ว (อย่าเสนอวิธีอื่นถ้าไม่จำเป็น)
1. `robot_state_publisher` ขึ้น "Unable to parse robot_description as yaml" แก้ด้วย `ParameterValue(Command(...), value_type=str)`
2. ของที่วางผ่าน GUI ของ Gazebo จะหายเมื่อปิดโปรแกรม และ Fortress ไม่มีปุ่ม scale ให้แก้ในไฟล์ `.sdf` โดยตรงเท่านั้น
3. `<camera>` ใน Fortress ต้องตั้ง `<clip><far>` ≥ 0.1 ถ้าตั้งต่ำกว่านี้ SDF ทั้งไฟล์จะ parse ไม่ผ่านและ spawn หุ่นไม่ได้ (Quad RGB ตั้งไว้ที่ 0.15)
4. `<include><uri>` แบบ relative ใน world จะเทียบกับ CWD ไม่ใช่ตำแหน่งไฟล์ world แก้โดยให้ launch ตั้ง `SDF_PATH` ชี้ไป `worlds/` ก่อน include gz_sim
5. segment ของชื่อ topic ห้ามขึ้นต้นด้วยตัวเลข (`/quad_rgb/1/image` ผิด) ฝั่ง `ros_gz_bridge` จะขึ้นแค่ `WARN: Failed to create a bridge` ซึ่งดูไม่เหมือน error ต้องอ่าน WARN ด้วยทุกครั้ง

## เกี่ยวกับผู้ใช้
ผู้ใช้เป็นมือใหม่ ให้อธิบายเป็นภาษาไทยทีละขั้นโดยไม่ข้ามขั้นตอน และเตือนเรื่อง `source install/setup.bash` ที่ต้องรันในทุกเทอร์มินัลใหม่
