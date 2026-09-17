import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from mbot2_control.modules import (
    QuadRGBArray,
    UltrasonicSensor,
    WheelEncoder,
)

# ความเร็ว/มุมเลี้ยวพื้นฐาน (ปรับตัวเลขพวกนี้เพื่อจูนพฤติกรรมได้เลย)
FORWARD_SPEED = 0.15

INITIAL_TURN_SPEED = -0.5      # ลบ = เลี้ยวขวา ตอนเจอเส้นครั้งแรก
INITIAL_TURN_DURATION = 1.0    # วินาที

TURN_GAIN = 0.15               # ยิ่งมาก ยิ่งเลี้ยวแรงตามระดับความเอียง
MAX_TURN = 0.5                 # เพดานความเร็วเลี้ยว กันหมุนแรงเกินไป

SEARCH_TURN_SPEED = 0.4        # ความแรงตอนหมุนหาเส้นที่หายไป

# หยุดเมื่อ Ultrasonic พบวัตถุด้านหน้าใกล้กว่าหรือเท่ากับระยะนี้
OBSTACLE_STOP_DISTANCE_M = 0.25

# ชื่อสถานะ ใช้ string ธรรมดาให้อ่าน log ง่าย
STATE_SEARCH = 'SEARCH'                # ยังไม่เคยเจอเส้นเลย เดินตรงไปเรื่อยๆ
STATE_INITIAL_TURN = 'INITIAL_TURN'    # เพิ่งเจอเส้นครั้งแรก เลี้ยวขวาก่อน
STATE_FOLLOW = 'FOLLOW'                # เดินตามเส้นปกติ
STATE_LINE_LOST = 'LINE_LOST'          # เดินตามอยู่ดีๆ เส้นหาย ต้องหมุนหา


class MazeSolver(Node):

    def __init__(self):
        super().__init__('maze_solver')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)

        # ดึงตัวช่วยอ่านเซนเซอร์ + encoder มาจาก modules/ (ไม่ต้องเขียน subscriber เองในนี้)
        self.sensors = QuadRGBArray(self)
        self.ultrasonic = UltrasonicSensor(self)
        self.encoder = WheelEncoder(self)  # ยังไม่ได้ใช้ตัดสินใจตอนนี้ เก็บไว้ต่อยอด

        self.state = STATE_SEARCH
        self.state_start_time = time.time()
        self.obstacle_stop_start = None

        self.get_logger().info(f'เริ่มทำงาน สถานะ: {self.state}')
        self.get_logger().info(
            'Ultrasonic จะหยุดหุ่นเมื่อพบวัตถุในระยะ '
            f'{OBSTACLE_STOP_DISTANCE_M:.2f} เมตร')

    def tick(self):
        cmd = Twist()

        # ให้ความปลอดภัยจาก Ultrasonic มาก่อนการเดินตามเส้นทุกสถานะ
        if self.ultrasonic.obstacle_ahead(OBSTACLE_STOP_DISTANCE_M):
            if self.obstacle_stop_start is None:
                self.obstacle_stop_start = time.time()
                self.get_logger().warning(
                    'พบสิ่งกีดขวางด้านหน้า '
                    f'{self.ultrasonic.distance_m:.2f} เมตร: หยุดหุ่น')

            self.cmd_pub.publish(cmd)
            return

        if self.obstacle_stop_start is not None:
            stopped_duration = time.time() - self.obstacle_stop_start
            self.state_start_time += stopped_duration
            self.obstacle_stop_start = None
            self.get_logger().info(
                'ทางด้านหน้าโล่งแล้ว: ทำงานตามเส้นต่อ')

        if self.state == STATE_SEARCH:
            # ยังไม่เจอเส้น: เดินตรงไปเรื่อยๆ
            cmd.linear.x = FORWARD_SPEED
            cmd.angular.z = 0.0

            if self.sensors.any_black():
                self._change_state(STATE_INITIAL_TURN)

        elif self.state == STATE_INITIAL_TURN:
            # เพิ่งเจอเส้นครั้งแรก: เลี้ยวขวาก่อนตามที่ตั้งใจไว้
            cmd.linear.x = FORWARD_SPEED
            cmd.angular.z = INITIAL_TURN_SPEED

            if time.time() - self.state_start_time >= INITIAL_TURN_DURATION:
                self._change_state(STATE_FOLLOW)

        elif self.state == STATE_FOLLOW:
            if self.sensors.any_black():
                cmd.linear.x = FORWARD_SPEED
                cmd.angular.z = self._line_follow_turn()
            else:
                # เดินตามอยู่ดีๆ ไม่เจอเส้นแล้ว -> ไปโหมดหมุนหา
                self._change_state(STATE_LINE_LOST)

        elif self.state == STATE_LINE_LOST:
            # เส้นหาย: หมุนขวาทางเดียวไปเรื่อยๆ จนกว่าจะเจอ
            cmd.linear.x = 0.0
            cmd.angular.z = -SEARCH_TURN_SPEED

            if self.sensors.any_black():
                self._change_state(STATE_FOLLOW)

        self.cmd_pub.publish(cmd)

    def _line_follow_turn(self):
        """คำนวณความแรงเลี้ยว จากเซนเซอร์ตัวไหนเจอเส้นดำบ้าง

        แนวคิด: เซนเซอร์ที่เจอเส้น = หุ่นเอียงไปทางนั้น ต้องเลี้ยวเข้าหาด้าน
        นั้นเพื่อดึงเส้นกลับมาอยู่กลางลำตัว ยิ่งเป็นเซนเซอร์ตัวนอก (1 หรือ 4)
        ยิ่งเอียงมาก ต้องเลี้ยวแรงกว่าตัวใน (2 หรือ 3) เล็กน้อย
        (บวก = เลี้ยวซ้าย, ลบ = เลี้ยวขวา ตามธรรมเนียมเดิมของโปรเจกต์นี้)
        """
        # เรียงตรงกับ self.sensors.black: [ซ้ายสุด, ซ้ายใน, ขวาใน, ขวาสุด]
        weights = [2, 1, -1, -2]
        active = [w for w, black in zip(weights, self.sensors.black) if black]

        if not active:
            return 0.0

        error = sum(active) / len(active)
        turn = TURN_GAIN * error
        return max(-MAX_TURN, min(MAX_TURN, turn))

    def _change_state(self, new_state):
        self.get_logger().info(f'{self.state} -> {new_state}')
        self.state = new_state
        self.state_start_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = MazeSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
