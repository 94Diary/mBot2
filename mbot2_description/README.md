# mBot2 บน Gazebo — คู่มือฉบับสมบูรณ์

คู่มือนี้พาไปตั้งแต่ **เช็คว่าเครื่องพร้อมหรือยัง** จนถึง **เห็นหุ่นยนต์ mBot2 วิ่งอยู่ใน Gazebo พร้อมเซนเซอร์ใช้งานได้จริง**
เขียนสำหรับ **Windows + WSL2 (Ubuntu-22.04) + ROS 2 Humble + Gazebo Sim Fortress**

> ทุกคำสั่งในเอกสารนี้พิมพ์ใน **เทอร์มินัล "Ubuntu"** (ไม่ใช่ PowerShell / CMD ของ Windows)

---

## สารบัญ

0. [เช็คว่าเครื่องพร้อมหรือยัง](#0-เช็คว่าเครื่องพร้อมหรือยัง)
1. [เตรียม ROS 2 workspace](#1-เตรียม-ros-2-workspace)
2. [ติดตั้งแพ็กเกจที่โมเดลนี้ต้องใช้](#2-ติดตั้งแพ็กเกจที่โมเดลนี้ต้องใช้)
3. [วางไฟล์โมเดลเข้า workspace](#3-วางไฟล์โมเดลเข้า-workspace)
4. [Build](#4-build)
5. [เปิดจำลอง](#5-เปิดจำลอง)
6. [สั่งให้หุ่นวิ่ง](#6-สั่งให้หุ่นวิ่ง)
7. [ดูค่าจากเซนเซอร์](#7-ดูค่าจากเซนเซอร์)
8. [(เสริม) ต่อ VS Code เพื่อแก้ไฟล์](#8-เสริม-ต่อ-vs-code-เพื่อแก้ไฟล์)
9. [แก้ปัญหาที่เจอบ่อย](#9-แก้ปัญหาที่เจอบ่อย)
- [ภาคผนวก: ถ้ายังไม่เคยติดตั้ง ROS 2 / Gazebo เลย](#ภาคผนวก-ถ้ายังไม่เคยติดตั้ง-ros-2--gazebo-เลย)

---

## 0. เช็คว่าเครื่องพร้อมหรือยัง

เปิดแอป **Ubuntu** (จาก Windows Search) แล้วรัน:

```bash
printenv ROS_DISTRO
```
ต้องขึ้น `humble`

```bash
ign gazebo --version
```
ต้องขึ้น `Gazebo Sim, version 6.x` (ตระกูล Fortress)

ถ้าคำสั่งไหนไม่มี output หรือขึ้น `command not found` แปลว่ายังไม่ได้ติดตั้ง — ข้ามไปทำ [ภาคผนวก](#ภาคผนวก-ถ้ายังไม่เคยติดตั้ง-ros-2--gazebo-เลย) ก่อน แล้วค่อยกลับมาเริ่มข้อ 1

**ทดสอบว่าเปิดหน้าต่างกราฟิกได้ไหม** (สำคัญมาก ถ้าข้อนี้ไม่ผ่านจะไม่เห็นตัวจำลองเลย):
```bash
ign gazebo -v 4 empty.sdf
```
ควรมีหน้าต่าง 3D เด้งขึ้นมาบน Desktop Windows เอง (ผ่านฟีเจอร์ WSLg) กด `Ctrl+C` ที่เทอร์มินัลเพื่อปิด

---

## 1. เตรียม ROS 2 workspace

```bash
mkdir -p ~/ros2_ws/src
```
โฟลเดอร์นี้คือที่เก็บ package ทั้งหมดที่คุณ build เอง สร้างครั้งเดียวพอ ไม่ต้องสร้างซ้ำอีก

---

## 2. ติดตั้งแพ็กเกจที่โมเดลนี้ต้องใช้

```bash
sudo apt update
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher \
                  ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
                  ros-humble-teleop-twist-keyboard unzip
```

---

## 3. วางไฟล์โมเดลเข้า workspace

1. ดาวน์โหลด `mbot2_description.zip` ที่ผมส่งให้ในแชท (ไปที่โฟลเดอร์ Downloads ของ Windows)
2. แตกไฟล์ตรงเข้า workspace ด้วยคำสั่งเดียว (ใช้ `-o` เพื่อทับไฟล์เก่าถ้าเคยมี):

```bash
unzip -o /mnt/c/Users/knkzu/Downloads/mbot2_description.zip -d ~/ros2_ws/src/
```

3. เช็คว่าวางถูกตำแหน่ง:
```bash
ls ~/ros2_ws/src/mbot2_description
```
ต้องเห็น `package.xml`, `CMakeLists.txt`, `urdf`, `launch`, `worlds`

---

## 4. Build

```bash
cd ~/ros2_ws
colcon build --packages-select mbot2_description
```
รอจนขึ้น `Summary: 1 package finished` โดยไม่มีตัวหนังสือสีแดง ก็ build สำเร็จ

---

## 5. เปิดจำลอง

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch mbot2_description gz_sim.launch.py
```

> ต้องรัน `source install/setup.bash` ทุกครั้งที่**เปิดเทอร์มินัลใหม่**แล้วจะใช้คำสั่ง `ros2` — ถ้าลืมจะขึ้น error ว่าหา package ไม่เจอ

คำสั่งนี้จะเปิด Gazebo พร้อมพื้นเปล่าๆ และหุ่นยนต์ mBot2 ยืนอยู่ตรงกลาง ปล่อยหน้าต่างนี้ไว้ **เปิดเทอร์มินัลใหม่อีกอัน** สำหรับขั้นตอนถัดไป (ไม่ต้องปิด Gazebo)

---

## 6. สั่งให้หุ่นวิ่ง

เปิดเทอร์มินัล Ubuntu **อีกหน้าต่างหนึ่ง** แล้ว `source install/setup.bash` ก่อน จากนั้นเลือกวิธีใดวิธีหนึ่ง:

**แบบพิมพ์คำสั่งตรงๆ:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.2}}" -r 10
```
กด `Ctrl+C` เพื่อหยุด

**แบบคุมด้วยคีย์บอร์ด:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
ใช้ปุ่มตามที่หน้าจอบอก (i/j/k/l ฯลฯ) บังคับหุ่นได้แบบเรียลไทม์

ดูตำแหน่งปัจจุบันของหุ่น:
```bash
ros2 topic echo /odom
```

---

## 7. ดูค่าจากเซนเซอร์

หุ่นมีเซนเซอร์อัลตราโซนิก (วัดระยะ 5-300 ซม.) และ IMU (วัดการหมุน/เอียง) ใช้งานได้จริง:

```bash
ros2 topic echo /ultrasonic/scan
```
ลองเอาของไปวางหน้าหุ่นในซิม หรือขับหุ่นเข้าใกล้กำแพง — ดูค่า `ranges[0]` เปลี่ยนไป

```bash
ros2 topic echo /imu
```
ลองสั่งหมุนหุ่น (`angular.z`) แล้วดูค่าเปลี่ยนไป

### สรุป Topic ทั้งหมด

| Topic | ชนิดข้อมูล | คืออะไร |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ส่งเข้าไปสั่งให้หุ่นเดิน/เลี้ยว |
| `/odom` | `nav_msgs/msg/Odometry` | ตำแหน่ง/ความเร็วปัจจุบันของหุ่น |
| `/joint_states` | `sensor_msgs/msg/JointState` | มุมหมุนของล้อ |
| `/ultrasonic/scan` | `sensor_msgs/msg/LaserScan` | ระยะจากเซนเซอร์อัลตราโซนิก (`ranges[0]`) |
| `/imu` | `sensor_msgs/msg/Imu` | การหมุน/ความเร่งจาก IMU บน CyberPi |
| `/tf` | `tf2_msgs/msg/TFMessage` | ตำแหน่งของทุกชิ้นส่วนหุ่น |

---

## 8. (เสริม) ต่อ VS Code เพื่อแก้ไฟล์

ไม่จำเป็นสำหรับแค่ดูหุ่นวิ่ง แต่สะดวกถ้าจะแก้โมเดลหรือเขียนโค้ดเพิ่ม:

1. ติดตั้ง VS Code บน Windows + extension ชื่อ **WSL** (`ms-vscode-remote.remote-wsl`)
2. ในเทอร์มินัล Ubuntu: `cd ~/ros2_ws/src/mbot2_description` แล้วพิมพ์ `code .`
3. VS Code จะเปิดเชื่อมกับ WSL อัตโนมัติ (มุมซ้ายล่างขึ้น `WSL: Ubuntu`)
4. แก้ไฟล์แล้ว save → กลับไป terminal สั่ง `colcon build --packages-select mbot2_description` ใหม่ → รัน `ros2 launch ...` ใหม่อีกรอบ (ไม่มีการอัปเดตสดอัตโนมัติ)

---

## 9. แก้ปัญหาที่เจอบ่อย

| อาการ | สาเหตุที่เป็นไปได้ | วิธีแก้ |
|---|---|---|
| `cd: No such file or directory` | พิมพ์ชื่อโฟลเดอร์ผิด (เช่น `ros_ws` แทน `ros2_ws`) | เช็คด้วย `ls ~/` ว่ามีโฟลเดอร์ชื่ออะไรจริง |
| `gazebo: command not found` | เครื่องนี้ใช้ Gazebo Fortress ไม่ใช่ Classic | ใช้ `ign gazebo` แทน `gazebo` |
| `Package 'mbot2_description' not found` | ยังไม่ได้ `colcon build` หรือลืม `source install/setup.bash` | กลับไปทำข้อ 4 แล้วข้อ 5 ตามลำดับ ทุกเทอร์มินัลใหม่ต้อง source ใหม่เสมอ |
| หน้าต่าง Gazebo ไม่ขึ้นเลย/ค้าง | ปัญหา GPU render ใน WSL | รัน `export LIBGL_ALWAYS_SOFTWARE=1` ก่อนคำสั่ง launch (บังคับ render ด้วย CPU แทน) |
| build error ว่าหา `ros_gz_sim`/`xacro` ไม่เจอ | ยังไม่ได้ลงแพ็กเกจในข้อ 2 | กลับไปรันคำสั่ง `apt install` ในข้อ 2 |
| `unzip: command not found` | ยังไม่ได้ลง unzip | `sudo apt install unzip` |

---

## ภาคผนวก: ถ้ายังไม่เคยติดตั้ง ROS 2 / Gazebo เลย

```bash
# locale
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# universe repo
sudo apt install software-properties-common -y
sudo add-apt-repository universe

# ROS 2 apt source
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ติดตั้ง ROS 2 Humble (เต็ม พร้อม GUI/จำลอง)
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop -y
sudo apt install ros-dev-tools -y

# โหลด ROS 2 อัตโนมัติทุกครั้งที่เปิดเทอร์มินัล
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# ติดตั้ง Gazebo Fortress + ตัวเชื่อม ROS 2
sudo apt install ros-humble-ros-gz -y

# เครื่องมือ build
sudo apt install python3-colcon-common-extensions -y
```

ติดตั้งเสร็จแล้ว กลับไปเริ่มที่ [ข้อ 0](#0-เช็คว่าเครื่องพร้อมหรือยัง) ของคู่มือนี้ได้เลย
