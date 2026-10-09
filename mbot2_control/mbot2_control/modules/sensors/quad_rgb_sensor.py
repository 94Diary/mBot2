"""อ่านค่าสีจากเซนเซอร์ Quad RGB 4 ตัวใต้ท้องหุ่น (ใช้เดินตามเส้น)"""

import time

from sensor_msgs.msg import Image

# ลำดับเซนเซอร์ตามที่วางไว้ใน urdf: 1=ซ้ายสุด, 2=ซ้ายใน, 3=ขวาใน, 4=ขวาสุด
# ชื่อไม่มี / นำหน้า เพื่อให้ ROS namespace ของหุ่นถูกเติมให้อัตโนมัติ
TOPICS = [
    'quad_rgb_1/image',
    'quad_rgb_2/image',
    'quad_rgb_3/image',
    'quad_rgb_4/image',
]


def image_to_rgb(msg: Image):
    """แปลง sensor_msgs/Image (1x1 พิกเซล, R8G8B8) เป็น (r, g, b)"""
    r, g, b = msg.data[0], msg.data[1], msg.data[2]
    return r, g, b


def is_black(r, g, b, threshold=60):
    """คืนค่า True ถ้าสีที่อ่านได้มืดพอจะถือว่าเป็นเส้นดำ"""
    return r < threshold and g < threshold and b < threshold


def is_yellow(r, g, b):
    """เหลือง: แดงและเขียวเด่น แต่น้ำเงินต่ำ (ปรับเกณฑ์หลังดูค่า RGB จริง)."""
    return r >= 110 and g >= 80 and b <= 90 and r > b * 1.5 and g > b * 1.5


def is_green(r, g, b):
    """เขียวเป้าหมาย: ช่องเขียวเด่นกว่าช่องแดงและน้ำเงิน."""
    return g >= 100 and g > r * 1.6 and g > b * 1.6


class QuadRGBArray:
    """สมัคร subscriber ให้ครบ 4 ตัว แล้วเก็บผลลัพธ์ล่าสุดไว้ให้เรียกใช้ง่ายๆ

    ใช้แบบนี้:
        self.sensors = QuadRGBArray(self)   # self คือ Node
        ...
        if self.sensors.any_black(): ...
    """

    def __init__(self, node, threshold=60):
        self.threshold = threshold
        # index 0=เซนเซอร์1(ซ้ายสุด) ... 3=เซนเซอร์4(ขวาสุด)
        self.black = [False, False, False, False]
        self.yellow = [False, False, False, False]
        self.green = [False, False, False, False]
        self.seen = [False, False, False, False]
        self.last_seen_at = [0.0, 0.0, 0.0, 0.0]
        self.rgb = [(0, 0, 0)] * 4

        for index, topic in enumerate(TOPICS):
            node.create_subscription(
                Image, topic, self._make_callback(index), 10)

    def _make_callback(self, index):
        def callback(msg):
            if len(msg.data) < 3:
                return
            r, g, b = image_to_rgb(msg)
            self.rgb[index] = (r, g, b)
            self.black[index] = is_black(r, g, b, self.threshold)
            self.yellow[index] = is_yellow(r, g, b)
            self.green[index] = is_green(r, g, b)
            self.seen[index] = True
            self.last_seen_at[index] = time.monotonic()
        return callback

    @property
    def ready(self):
        """รอภาพจากทั้ง 4 ตัวก่อนเริ่มขับ เพื่อลดการตัดสินใจจากค่าเริ่มต้น."""
        return all(self.seen)

    def fresh(self, max_age_seconds):
        """True only while all four camera streams are still arriving."""
        now = time.monotonic()
        return self.ready and all(
            now - last_seen <= max_age_seconds for last_seen in self.last_seen_at)

    @property
    def left_outer(self):
        return self.black[0]

    @property
    def left_inner(self):
        return self.black[1]

    @property
    def right_inner(self):
        return self.black[2]

    @property
    def right_outer(self):
        return self.black[3]

    def any_black(self):
        return any(self.black)

    def any_yellow(self):
        return any(self.yellow)

    def any_green(self):
        return any(self.green)

    def junction_sides(self):
        """สังเกตเส้นแยกจากตัวนอก ขณะตัวในยังเกาะเส้นหลัก.

        คืน (ซ้าย, ขวา) เป็นเพียงการสังเกต ณ ขณะนั้น ไม่ใช่แผนที่ถาวร.
        ต้องใช้ร่วมกับแถบเหลืองก่อนแยกเพื่อไม่ให้เส้นตรงถูกเข้าใจผิด.
        """
        on_main_line = self.black[1] and self.black[2]
        return on_main_line and self.black[0], on_main_line and self.black[3]
