# mBot2 บน ROS 2 และ Gazebo Sim

โปรเจกต์นี้จำลองหุ่นยนต์ mBot2 ใน Gazebo Sim Fortress บน ROS 2 Humble หุ่นมี Quad RGB สำหรับตามเส้น และ Ultrasonic สำหรับหยุดเมื่อมีสิ่งกีดขวางด้านหน้า

รองรับหุ่น 1 ตัว และหุ่นหลายตัวในโลก Gazebo เดียวกัน โดยแยกข้อมูลของแต่ละตัวด้วย ROS namespace

ทุกคำสั่งในเอกสารนี้ให้พิมพ์ใน Ubuntu terminal ของ WSL ไม่ใช่ PowerShell หรือ Command Prompt

## สิ่งที่ต้องมี

- Windows 11 และ WSL2
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Sim Fortress
- workspace อยู่ที่ ~/ros2_ws

เปิด source code ใน Windows Explorer ได้ที่:

~~~
\\wsl.localhost\Ubuntu-22.04\home\knkzu\ros2_ws\src
~~~

## เริ่มเร็ว: เปิด mBot2 สองตัว

คำสั่งนี้ทำทุกอย่างให้แล้ว: เปิด Gazebo, สร้าง mBot1/mBot2, สร้าง bridge และเริ่ม maze_solver ให้ทั้งคู่

~~~bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mbot2_control mbot2_description --symlink-install
source install/setup.bash
ros2 launch mbot2_description two_mbot2.launch.py
~~~

ถ้าทำงานสำเร็จ จะเห็น Gazebo ที่มีหุ่น 2 ตัว และ log คล้ายนี้:

~~~
[mbot1.maze_solver]: เริ่มทำงาน สถานะ: SEARCH
[mbot2.maze_solver]: เริ่มทำงาน สถานะ: SEARCH
~~~

สำคัญ: หลังใช้ two_mbot2.launch.py แล้ว ไม่ต้องรัน ros2 run mbot2_control maze_solver ซ้ำ เพราะ launch นี้เริ่ม controller ของทั้งสองตัวแล้ว

กด Ctrl+C ใน terminal ที่ใช้ launch เพื่อปิด Gazebo, bridge และ controllers ทั้งหมด

## คำสั่งเดิม: หุ่น 1 ตัว

หากต้องการหุ่นเดี่ยว ให้เปิด 2 terminals

Terminal 1:

~~~bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch mbot2_description gz_sim.launch.py
~~~

Terminal 2:

~~~bash
source ~/ros2_ws/install/setup.bash
ros2 run mbot2_control maze_solver
~~~

## Flow chart

~~~mermaid
flowchart LR
  World[Gazebo world] --> M1[mBot1 model]
  World --> M2[mBot2 model]
  M1 --> S1["/mbot1 Quad RGB and Ultrasonic"]
  M2 --> S2["/mbot2 Quad RGB and Ultrasonic"]
  S1 --> C1["/mbot1/maze_solver"]
  S2 --> C2["/mbot2/maze_solver"]
  C1 --> V1["/mbot1/cmd_vel"]
  C2 --> V2["/mbot2/cmd_vel"]
  V1 --> M1
  V2 --> M2
~~~

ลำดับการทำงาน:

1. Gazebo จำลองหุ่นและเซ็นเซอร์
2. ros_gz_bridge แปลงข้อมูล Gazebo เป็น ROS 2 topics
3. maze_solver ของแต่ละตัวอ่านเซ็นเซอร์ของตัวเอง
4. maze_solver ส่งความเร็วไปยัง cmd_vel ของหุ่นตัวเอง
5. Gazebo รับคำสั่งและหมุนล้อของหุ่นที่ถูกต้อง

## Topic คืออะไร

Topic คือช่องสื่อสารของ ROS 2 คล้ายช่องวิทยุ

- publisher คือโปรแกรมที่ส่งข้อมูล
- subscriber คือโปรแกรมที่รับข้อมูล
- โปรแกรมที่ใช้ชื่อ topic เดียวกันจะสื่อสารกันได้

ตัวอย่าง maze_solver ส่งคำสั่งความเร็วไปให้ Gazebo:

~~~
maze_solver -> cmd_vel -> Gazebo DiffDrive -> ล้อหุ่น
~~~

ถ้าหุ่นสองตัวใช้ชื่อ cmd_vel เดียวกัน ทั้งคู่จะรับคำสั่งปนกัน จึงแบ่ง namespace:

~~~
/mbot1/cmd_vel  สำหรับ mBot1
/mbot2/cmd_vel  สำหรับ mBot2
~~~

Namespace คือชื่อโฟลเดอร์ที่วางอยู่หน้าชื่อ topic เช่น /mbot1 และ /mbot2

### ต้องสร้าง topic เองไหม

ไม่ต้อง Topic จะเกิดเองเมื่อ publisher หรือ subscriber เริ่มทำงาน

two_mbot2.launch.py ทำให้ครบแล้ว:

1. สร้างชื่อ sensor topics ใน URDF/Xacro
2. สร้าง bridge ระหว่าง Gazebo และ ROS 2
3. เปิด controller ใน namespace ที่ถูกต้อง

ถ้าเพิ่มเซ็นเซอร์ใหม่ภายหลัง ต้องแก้ครบ 3 จุด:

1. เพิ่ม sensor และชื่อ Gazebo topic ใน mbot2.urdf.xacro
2. เพิ่มชนิดข้อมูลใน bridge ของ two_mbot2.launch.py
3. เพิ่ม subscriber ใน mbot2_control/modules/sensors

## Topics สำคัญ

แทน <robot> ด้วย mbot1 หรือ mbot2

| Topic | ชนิดข้อมูล | หน้าที่ |
|---|---|---|
| /<robot>/cmd_vel | geometry_msgs/msg/Twist | คำสั่งเดินหน้าและเลี้ยว |
| /<robot>/odom | nav_msgs/msg/Odometry | ตำแหน่งและความเร็วจาก Gazebo |
| /<robot>/joint_states | sensor_msgs/msg/JointState | มุมล้อ เสมือน encoder |
| /<robot>/ultrasonic/scan | sensor_msgs/msg/LaserScan | ระยะด้านหน้าจาก Ultrasonic |
| /<robot>/imu | sensor_msgs/msg/Imu | การเร่งและการหมุนจาก IMU |
| /<robot>/quad_rgb_1/image ถึง 4/image | sensor_msgs/msg/Image | สีใต้หุ่นจาก Quad RGB |
| /<robot>/tf | tf2_msgs/msg/TFMessage | ตำแหน่งสัมพันธ์ของชิ้นส่วนหุ่น |

ดู topics ทั้งหมด:

~~~bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
~~~

ดู Ultrasonic:

~~~bash
ros2 topic echo /mbot1/ultrasonic/scan
ros2 topic echo /mbot2/ultrasonic/scan
~~~

ดู ROS nodes:

~~~bash
ros2 node list
~~~

ในโหมดสองหุ่นควรเห็น:

~~~
/mbot1/maze_solver
/mbot1/robot_state_publisher
/mbot2/maze_solver
/mbot2/robot_state_publisher
/two_mbot2_bridge
~~~

## อธิบายคำสั่ง

### cd ~/ros2_ws

cd ย่อจาก change directory คือเปลี่ยนโฟลเดอร์ปัจจุบัน

เครื่องหมาย ~ หมายถึง home directory ของ Ubuntu ดังนั้น ~/ros2_ws คือ /home/knkzu/ros2_ws

### source /opt/ros/humble/setup.bash

โหลด environment ของ ROS 2 Humble เช่นคำสั่ง ros2, package paths และ library paths ต้องทำทุกครั้งที่เปิด terminal ใหม่ หากยังไม่ได้ใส่ไว้ใน ~/.bashrc

### source install/setup.bash

โหลด packages ที่เรา build เองใน workspace เช่น mbot2_control และ mbot2_description

### colcon build

colcon คือเครื่องมือ build ของ ROS 2

~~~bash
colcon build --packages-select mbot2_control mbot2_description
~~~

หมายถึง build เฉพาะสอง packages นี้ ไม่ต้อง build ทุก package ใน workspace

### --symlink-install

ปกติ colcon จะ copy ไฟล์ Python ไปไว้ใน install ทำให้แก้ source แล้วต้อง build ซ้ำจึงจะเห็นผล

--symlink-install ให้ install ชี้กลับมาที่ source ด้วย symbolic link จึงสะดวกกับการแก้โค้ด Python

แม้ใช้ option นี้ ก็ควร build และ restart launch ใหม่เมื่อแก้ไฟล์เหล่านี้:

- launch/*.launch.py
- urdf/*.xacro
- worlds/*.sdf
- package.xml
- setup.py

### ros2 launch

เริ่มหลาย ROS processes ตามแผนใน launch file

~~~bash
ros2 launch mbot2_description two_mbot2.launch.py
~~~

คำสั่งนี้ไม่ได้เปิดแค่หน้าต่าง Gazebo แต่ยังเปิด bridge, robot_state_publisher, spawn robot และ maze_solver ของ mBot1/mBot2 ด้วย

### ros2 run

เริ่มโปรแกรม ROS 2 หนึ่งตัว:

~~~bash
ros2 run mbot2_control maze_solver
~~~

เหมาะกับโหมดหุ่นเดี่ยว

หาก Gazebo หลายหุ่นเปิดอยู่แต่ไม่มี controller ให้รันแบบ namespace:

~~~bash
ros2 run mbot2_control maze_solver --ros-args -r __ns:=/mbot1
ros2 run mbot2_control maze_solver --ros-args -r __ns:=/mbot2

~~~
~~~bash รันสองตัวพร้อมกัน
source ~/ros2_ws/install/setup.bash
ros2 run mbot2_control maze_solver --ros-args -r __ns:=/mbot1 &
ros2 run mbot2_control maze_solver --ros-args -r __ns:=/mbot2 &

wait
~~~

--ros-args คือ options ของ ROS 2 และ -r __ns:=/mbot1 คือบอกให้ node อยู่ใต้ namespace /mbot1

ห้ามใช้คำสั่งสองบรรทัดนี้ซ้ำกับ two_mbot2.launch.py เพราะ launch เปิด controller ทั้งสองตัวให้อยู่แล้ว

### ros2 topic echo

แสดง messages ที่ไหลบน topic แบบสด:

~~~bash
ros2 topic echo /mbot1/ultrasonic/scan
~~~

กด Ctrl+C เพื่อหยุดดู

### ros2 topic pub

ส่งคำสั่งทดสอบเอง ตัวอย่างสั่ง mBot1 เดินตรง:

~~~bash
ros2 topic pub --once /mbot1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
~~~

อย่าส่งคำสั่งนี้พร้อม maze_solver เพราะทั้งสองจะส่งความเร็วให้หุ่นตัวเดียวกัน ควรหยุด controller ก่อนเมื่อจะบังคับมือ

## โครงสร้างโฟลเดอร์

~~~
~/ros2_ws/
├── src/                              source code ที่แก้ด้วย VS Code
│   ├── mbot2_description/            โมเดลหุ่น, สนาม และ Gazebo launch
│   │   ├── launch/
│   │   │   ├── gz_sim.launch.py       หุ่นเดี่ยวแบบเดิม
│   │   │   └── two_mbot2.launch.py    หุ่นสองตัวพร้อม controllers
│   │   ├── urdf/
│   │   │   └── mbot2.urdf.xacro       โมเดล, sensors, Gazebo topics/plugins
│   │   └── worlds/
│   │       ├── mbot2_world.sdf        โลก Gazebo หลัก
│   │       └── test_platform.sdf      พื้นและเส้นดำ
│   └── mbot2_control/                 โค้ด Python ควบคุมหุ่น
│       └── mbot2_control/
│           ├── maze_solver.py         สมองเดินตามเส้นและหยุดสิ่งกีดขวาง
│           └── modules/
│               ├── sensors/
│               │   ├── quad_rgb_sensor.py
│               │   └── ultrasonic_sensor.py
│               └── utils/
│                   └── encoder.py
├── build/                             colcon สร้าง: ห้ามแก้
├── install/                           colcon สร้าง: ห้ามแก้
└── log/                               log ของ colcon: ห้ามแก้
~~~

แก้โค้ดใน src เท่านั้น

## ไฟล์ที่ต้องรู้

### mbot2_description/urdf/mbot2.urdf.xacro

โมเดลของหุ่น ประกอบด้วย chassis, ล้อ, CyberPi, Ultrasonic, Quad RGB และ Gazebo plugins

ไฟล์นี้รับ robot_namespace เช่น /mbot1 หรือ /mbot2 เพื่อสร้าง topics แยกกัน

### mbot2_description/launch/two_mbot2.launch.py

เป็นผู้จัดการระบบหลายหุ่น:

- กำหนดชื่อและตำแหน่งเริ่มต้นใน ROBOTS
- สร้าง URDF แยก namespace
- spawn models ใน Gazebo
- เปิด robot_state_publisher
- เปิด maze_solver ของแต่ละตัว
- เปิด bridge ครบทุก topics

### mbot2_control/mbot2_control/maze_solver.py

เป็นสมองของหุ่น:

1. อ่าน Quad RGB เพื่อหาเส้นดำ
2. อ่าน Ultrasonic เพื่อหาสิ่งกีดขวาง
3. หยุดเมื่อเจอวัตถุไม่เกิน 0.25 เมตร
4. ถ้าทางโล่ง กลับไปเดินตามเส้น
5. ส่งความเร็วออก cmd_vel

ค่าที่ลองปรับได้:

~~~python
FORWARD_SPEED = 0.15
OBSTACLE_STOP_DISTANCE_M = 0.25
~~~

### modules/sensors/quad_rgb_sensor.py

รับข้อมูล Quad RGB สี่ตัว แล้วบอกว่าแต่ละจุดเป็นสีดำหรือไม่

### modules/sensors/ultrasonic_sensor.py

รับ ultrasonic/scan เก็บระยะล่าสุดใน distance_m และมี function obstacle_ahead(0.25)

### modules/utils/encoder.py

รับ joint_states แล้วเก็บมุมล้อซ้าย/ขวา ใช้ต่อยอดคำนวณระยะทางได้

## เพิ่ม mBot3 หรือมากกว่า

เปิดไฟล์:

~~~
src/mbot2_description/launch/two_mbot2.launch.py
~~~

เพิ่มหนึ่งบรรทัดใน ROBOTS:

~~~python
ROBOTS = (
    ('mbot1', '-0.30', '-0.20', '1.5707963'),
    ('mbot2', '0.30', '0.20', '-1.5707963'),
    ('mbot3', '0.00', '0.00', '0.0'),
)
~~~

รูปแบบคือ:

~~~
('ชื่อหุ่น', 'ตำแหน่ง x เป็นเมตร', 'ตำแหน่ง y เป็นเมตร', 'มุม yaw เป็นเรเดียน')
~~~

จากนั้น build และ launch ใหม่:

~~~bash
cd ~/ros2_ws
colcon build --packages-select mbot2_description mbot2_control --symlink-install
source install/setup.bash
ros2 launch mbot2_description two_mbot2.launch.py
~~~

ไม่ต้องแก้ bridge_arguments เพราะมันวนลูปจาก ROBOTS และสร้าง topics ของ robot ใหม่ให้อัตโนมัติ

วางจุดเริ่มให้หุ่นไม่ซ้อนหรือชนกัน ถ้าหลายตัวใช้เส้นเดียวกัน Ultrasonic อาจพบหุ่นตัวข้างหน้าและสั่งหยุด ซึ่งเป็นพฤติกรรมที่ตั้งใจไว้

## หลังแก้โค้ด

1. Save ไฟล์ใน VS Code
2. กลับ terminal ที่ launch อยู่ กด Ctrl+C
3. build packages ที่เกี่ยวข้อง
4. source install/setup.bash
5. launch ใหม่

~~~bash
cd ~/ros2_ws
colcon build --packages-select mbot2_control mbot2_description --symlink-install
source install/setup.bash
ros2 launch mbot2_description two_mbot2.launch.py
~~~

## ปัญหาที่พบบ่อย

| อาการ | วิธีแก้ |
|---|---|
| Package not found | source ROS และ source install/setup.bash |
| แก้โค้ดแล้วไม่เปลี่ยน | build ใหม่, source ใหม่, restart launch |
| เห็นสองหุ่นแต่ไม่เดิน | ros2 node list ต้องเห็น maze_solver ของทั้งสองตัว |
| หุ่นขยับแปลก | อย่ารัน maze_solver ซ้ำ และอย่าส่ง topic pub ซ้อน |
| Gazebo render มีปัญหา | ลอง export LIBGL_ALWAYS_SOFTWARE=1 ก่อน launch |
| topics ปนกัน | ใช้ /mbot1/... หรือ /mbot2/... ให้ครบในโหมดหลายหุ่น |

## ลำดับการฝึกที่แนะนำ

1. รันหุ่นเดี่ยวและอ่าน maze_solver.py
2. ปรับ FORWARD_SPEED และ OBSTACLE_STOP_DISTANCE_M
3. ใช้ ros2 topic echo ดู Ultrasonic
4. รันสองหุ่นและสังเกต namespace
5. เพิ่ม mBot3 ใน ROBOTS
6. เพิ่ม logic หลบสิ่งกีดขวางแทนการหยุด
