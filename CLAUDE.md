# mBot2 Swarm Cave Exploration — Project Context

## เป้าหมายโปรเจกต์
สร้างฝูงหุ่นยนต์ mBot2 สำรวจถ้ำ/เขาวงกตอัตโนมัติ แต่ละตัวสื่อสารกันว่าสำรวจไปทางไหนแล้ว (ทางตัน/ทางที่ไปแล้ว) เพื่อไม่เดินซ้ำทาง แล้วช่วยกันหาทางออก

**ข้อจำกัดสำคัญที่สุด: ต้องเอาโค้ดไปลงเครื่องจริง (mBot2 ของจริง) ได้ ไม่ใช่จำลองอย่างเดียว**
→ ห้ามออกแบบอะไรที่ต้องพึ่งเซนเซอร์ที่ mBot2 จริงไม่มี (โดยเฉพาะ 2D LiDAR — ของจริงไม่มี มีแค่อัลตราโซนิกจุดเดียว)
→ อัลกอริทึมที่เลือกใช้คือ **Trémaux's algorithm** (เดินตามผนัง + จำทางที่เคยผ่าน) เพราะใช้แค่เซนเซอร์ระยะจุดเดียวได้ ไม่ต้องมี SLAM/แผนที่เต็ม
→ การแชร์ข้อมูลระหว่างหุ่น: ไม่ใช้ map merging (นั่นสำหรับ LiDAR) แต่ใช้วิธีง่ายกว่า — ส่งพิกัด+สถานะ (ทางตัน/สำรวจแล้ว) ไปที่ topic กลาง เช่น `/swarm/dead_ends` ให้ตัวอื่นเช็คก่อนตัดสินใจเดิน

## Environment (สำคัญ อย่าแนะนำอะไรที่ไม่ตรงกับนี้)
- Windows 11 + **WSL2 Ubuntu 22.04**
- **ROS 2 Humble**
- **Gazebo Sim Fortress** (v6.18.0) — เรียกผ่านคำสั่ง `ign gazebo` ไม่ใช่ `gazebo` (Classic) หรือ `gz` (เวอร์ชันใหม่กว่า Fortress)
- Plugin library names ใช้รูปแบบเก่า `libignition-gazebo-*-system.so` (ไม่ใช่ `libgz-sim-*` ของเวอร์ชันใหม่กว่า)
- **GPU render บน WSLg ใช้ไม่ได้กับ Gazebo Fortress เวอร์ชันนี้** (บั๊กที่รู้กันมานาน แก้แล้วเฉพาะ Garden/Harmonic) → ต้องมี `export LIBGL_ALWAYS_SOFTWARE=1` ใน `~/.bashrc` (ตั้งไว้แล้ว) ไม่งั้น Gazebo จะเปิดแล้วปิดตัวเอง (crash)
- Workspace: `~/ros2_ws/src/` (เปิดใน VS Code ผ่าน `code ~/ros2_ws/src`)
- Git repo: https://github.com/94Diary/mBot2.git (push ผ่าน CLI แล้ว ใช้ได้ปกติ — **GitHub Desktop ใช้กับ WSL ไม่ได้** เป็น known limitation ให้ใช้ VS Code Source Control panel หรือ CLI แทน)

## โครงสร้างแพ็กเกจ

### `mbot2_description/` — โมเดลหุ่น + world (อย่าสับสนกับ mbot2_control)
- `urdf/mbot2.urdf.xacro` — ไฟล์เดียวที่สร้างโมเดลทั้งหมด (รูปทรง/น้ำหนัก/แรงเสียดทาน/เซนเซอร์ทั้งหมดอยู่ในนี้)
- `launch/gz_sim.launch.py` — เปิด Gazebo + spawn หุ่น + bridge topics (มี `world` launch arg เลือก world ได้)
- `worlds/mbot2_maze_world.sdf` — world เริ่มต้น: ทางเข้า → ทางแยก → วงวนรอบเกาะกลาง → แยกทางตัน/ทางออก มีเส้นดำ+แพทช์สีเขียวไว้ทดสอบ Quad RGB ด้วย
- `worlds/mbot2_world.sdf` — world เปล่า (พื้นเปล่าไม่มีกำแพง) เผื่อทดสอบอย่างอื่น

**เซนเซอร์ที่มีในโมเดล (ตรงกับของจริง mBot2 ทุกตัว — ห้ามเพิ่ม LiDAR):**
| เซนเซอร์ | Topic | ชนิดข้อมูล |
|---|---|---|
| Ultrasonic (จุดเดียว, 5-300cm) | `/ultrasonic/scan` | `sensor_msgs/LaserScan` (ใช้แค่ `ranges[0]`) |
| IMU (gyro/accel บน CyberPi) | `/imu` | `sensor_msgs/Imu` |
| Quad RGB ×4 (กล้อง 1x1 พิกเซล ใต้ท้อง) | `/quad_rgb/1-4/image` | `sensor_msgs/Image` |
| ขับเคลื่อน (diff-drive) | `/cmd_vel` (สั่งเข้า), `/odom` (อ่านออก) | `Twist`, `Odometry` |

### `mbot2_control/` — โค้ดควบคุม (ament_python package)
- `mbot2_control/maze_solver.py` — ตอนนี้เป็นแค่ demo เดินตามลำดับเวลาที่กำหนดตายตัว (เดินหน้า/ถอย/เลี้ยว) **ยังไม่ใช่ Trémaux's algorithm จริง** — ขั้นต่อไปคือเขียนตัวนี้ให้เป็น Trémaux's จริง โดยใช้ `/ultrasonic/scan` ตัดสินใจ
- entry point ต้องเพิ่มใน `setup.py` (`entry_points/console_scripts`) ทุกครั้งที่สร้างไฟล์ใหม่ที่อยากสั่ง `ros2 run` ได้ตรงๆ
- แผนไว้ว่าจะแยกโค้ดใช้ร่วมไปไว้ `mbot2_control/mbot2_control/modules/` เมื่อโค้ดเริ่มยาว (ยังไม่ได้สร้างจริง)

## วิธีรันทดสอบ (ต้องมี 2 เทอร์มินัลพร้อมกันเสมอ)
```bash
# เทอร์มินัล 1
cd ~/ros2_ws && source install/setup.bash
ros2 launch mbot2_description gz_sim.launch.py

# เทอร์มินัล 2
source ~/ros2_ws/install/setup.bash
ros2 run mbot2_control maze_solver
```
แก้โค้ดแล้วต้อง: save → Ctrl+C ที่เทอร์มินัลที่รันอยู่ → `colcon build --packages-select <ชื่อแพ็กเกจ>` → รันใหม่ (ไม่มี hot-reload)

## บั๊ก/ข้อจำกัดที่เจอมาแล้วและวิธีแก้ (อย่าเสนอวิธีแก้แบบอื่นถ้าไม่จำเป็น)
1. **`robot_state_publisher` error "Unable to parse robot_description as yaml"** → แก้แล้วด้วย `ParameterValue(Command(...), value_type=str)` ใน launch file
2. **Gazebo เปิดแล้วปิดเอง (เด้ง)** → GPU render พังบน WSLg+Fortress คู่นี้ → ใช้ `LIBGL_ALWAYS_SOFTWARE=1` (ตั้งถาวรใน `.bashrc` แล้ว)
3. **กำแพง/สิ่งของที่ลากวางในหน้าต่าง Gazebo ตอนรันอยู่ (ผ่านเมนู insert) จะหายตอนปิดโปรแกรม** — ต้องเขียนลงไฟล์ `.sdf` ตรงๆ ถึงจะถาวร (ที่กำแพง maze ถาวร เพราะเขียนลงไฟล์)
4. **Gazebo Fortress ไม่มีปุ่ม resize/scale วัตถุใน GUI** (มีแค่ translate/rotate) ต้องแก้ขนาดใน SDF ไฟล์โดยตรง
5. **ผู้ใช้เป็นมือใหม่มาก** ต้องอธิบายละเอียด ทีละขั้น เป็นภาษาไทย ไม่ข้ามขั้นตอน ระวังเรื่อง `source install/setup.bash` ที่ต้องรันทุกเทอร์มินัลใหม่ (ลืมบ่อย)
6. **`<camera>` sensor ใน Gazebo Fortress ต้องตั้งค่า `<clip><far>` ไม่ต่ำกว่า 0.1 เมตร** ไม่งั้น Gazebo อ่าน SDF ทั้งไฟล์ไม่ผ่าน (error `The value [x] is less than the minimum allowed value of [0.1] for key [far]` ลามจน spawn หุ่นไม่ได้เลย) — เจอตอนทำ Quad RGB sensor (กล้อง 1x1 พิกเซลมองพื้นใกล้ๆ) ที่ตอนแรกตั้ง `far` ไว้ 0.05 เพราะคิดว่าเซนเซอร์อยู่ใกล้พื้นแค่ ~2 ซม. → แก้โดยตั้ง `far` เป็น 0.15 แทน (ยังมองเห็นพื้นที่ระยะจริงได้ตามปกติ ค่า near/far แค่กำหนดช่วงมองเห็น ไม่ใช่ระยะที่ต้องอยู่พอดี)

## Toolchain เฉพาะโปรเจกต์นี้
- `xacro` แปลง `.urdf.xacro` → `.urdf`
- `colcon build --packages-select <ชื่อ>` ทุกครั้งที่แก้โค้ด/โมเดล
- `ros2 pkg create --build-type ament_python <ชื่อ> --dependencies rclpy geometry_msgs sensor_msgs` สำหรับสร้างแพ็กเกจ Python ใหม่

## สิ่งที่ยังไม่ได้ทำ (Next steps)
1. เขียน Trémaux's algorithm จริงใน `maze_solver.py` (ตอนนี้เป็นแค่ demo ตามเวลา)
2. เพิ่ม node สื่อสารระหว่างหุ่น (`/swarm/dead_ends` หรือชื่อคล้ายกัน)
3. ทำให้ spawn ได้หลายตัวพร้อมกัน (ตอนนี้ launch file รองรับแค่ตัวเดียว ต้องเพิ่ม namespace ต่อหุ่น)
4. แยกโค้ดใช้ร่วมไปไว้ `modules/` เมื่อไฟล์เริ่มยาว
