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
    """เหลือง: รับค่าที่ Gazebo วัดได้จริง เช่น (100, 87, 0) บนแถบเตือน."""
    return r >= 85 and g >= 65 and b <= 90 and r > b * 1.5 and g > b * 1.5


def is_green(r, g, b):
    """เขียวบนแผนที่ใหม่เป็นจุดแยก."""
    return g >= 100 and g > r * 1.6 and g > b * 1.6


def is_red(r, g, b):
    """แดงบนแผนที่ใหม่เป็นเป้าหมาย."""
    return r >= 100 and r > g * 1.6 and r > b * 1.6


def is_blue(r, g, b):
    """ฟ้าคือพื้นที่เริ่มต้น ไม่ใช่เส้นทางหรือทางแยก."""
    return b >= 100 and b > r * 1.6 and b > g * 1.4


class QuadRGBArray:
    """สมัคร subscriber ให้ครบ 4 ตัว แล้วเก็บผลลัพธ์ล่าสุดไว้ให้เรียกใช้ง่ายๆ

    ใช้แบบนี้:
        self.sensors = QuadRGBArray(self)   # self คือ Node
        ...
        if self.sensors.any_black(): ...
    """

    def __init__(self, node, threshold=60):
        self.threshold = threshold
        self.junction_color = node.get_parameter('junction_color').value
        self.goal_color = node.get_parameter('goal_color').value
        # index 0=เซนเซอร์1(ซ้ายสุด) ... 3=เซนเซอร์4(ขวาสุด)
        self.black = [False, False, False, False]
        self.yellow = [False, False, False, False]
        self.green = [False, False, False, False]
        self.blue = [False, False, False, False]
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
            self.yellow[index] = (is_green(r, g, b) if self.junction_color == 'green'
                                  else is_yellow(r, g, b))
            self.green[index] = (is_red(r, g, b) if self.goal_color == 'red'
                                 else is_green(r, g, b))
            self.blue[index] = is_blue(r, g, b)
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

    def any_blue(self):
        return any(self.blue)

    def junction_sides(self):
        """อ่านกิ่งซ้าย/ขวาจากตัวนอกเมื่อเซนเซอร์ในอยู่บนจุดเหลือง.

        สีเหลืองตรงกลางแยกบังเส้นดำใต้ตัวใน จึงห้ามบังคับให้ตัวใน
        เห็นดำพร้อมกัน การเรียกฟังก์ชันนี้ต้องตรวจ yellow ก่อนเสมอ.
        แผนที่ใหม่ทาสีแยกทับกิ่งข้างด้วย ตัวนอกจึงอาจเห็นสีแยกแทนดำ.
        """
        # Only at the cross itself are both inner sensors on path colour; an
        # off-centre robot on the inbound arm touches it with one outer sensor.
        inner_on_path = all(self.black[i] or self.yellow[i] for i in (1, 2))
        return (self.black[0] or (inner_on_path and self.yellow[0]),
                self.black[3] or (inner_on_path and self.yellow[3]))
