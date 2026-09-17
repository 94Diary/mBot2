"""อ่านระยะด้านหน้าจากเซ็นเซอร์ Ultrasonic ของ mBot2."""

import math

from sensor_msgs.msg import LaserScan

# ชื่อไม่มี / นำหน้า เพื่อให้ ROS namespace ของหุ่นถูกเติมให้อัตโนมัติ
TOPIC = 'ultrasonic/scan'


def nearest_valid_range(msg: LaserScan):
    """คืนระยะที่ใกล้ที่สุดเป็นเมตร หรือ None ถ้ายังไม่มีค่าที่ใช้ได้."""
    valid_ranges = [
        distance for distance in msg.ranges
        if math.isfinite(distance)
        and distance >= msg.range_min
        and distance <= msg.range_max
    ]

    if not valid_ranges:
        return None

    return min(valid_ranges)


class UltrasonicSensor:
    """
    เก็บค่าระยะล่าสุดและช่วยตรวจสิ่งกีดขวางด้านหน้าหุ่น.

    ใช้แบบนี้:
        self.ultrasonic = UltrasonicSensor(self)
        ...
        if self.ultrasonic.obstacle_ahead(0.25):
            # พบสิ่งกีดขวางในระยะ 25 เซนติเมตร
            ...
    """

    def __init__(self, node, topic=TOPIC):
        """สมัครรับ LaserScan และเตรียมพื้นที่เก็บระยะล่าสุด."""
        self.distance_m = None
        self.ready = False

        self._subscription = node.create_subscription(
            LaserScan,
            topic,
            self._callback,
            10,
        )

    def _callback(self, msg: LaserScan):
        self.ready = True
        self.distance_m = nearest_valid_range(msg)

    def obstacle_ahead(self, stop_distance_m=0.25):
        """คืน True เมื่อมีวัตถุอยู่ไม่เกินระยะหยุดที่กำหนด."""
        if stop_distance_m <= 0.0:
            raise ValueError('stop_distance_m must be greater than zero')

        if not self.ready or self.distance_m is None:
            return False

        return self.distance_m <= stop_distance_m
