"""อ่านค่าสีจากเซนเซอร์ Quad RGB 4 ตัวใต้ท้องหุ่น (ใช้เดินตามเส้น)"""

from sensor_msgs.msg import Image

# ลำดับเซนเซอร์ตามที่วางไว้ใน urdf: 1=ซ้ายสุด, 2=ซ้ายใน, 3=ขวาใน, 4=ขวาสุด
TOPICS = [
    '/quad_rgb_1/image',
    '/quad_rgb_2/image',
    '/quad_rgb_3/image',
    '/quad_rgb_4/image',
]


def image_to_rgb(msg: Image):
    """แปลง sensor_msgs/Image (1x1 พิกเซล, R8G8B8) เป็น (r, g, b)"""
    r, g, b = msg.data[0], msg.data[1], msg.data[2]
    return r, g, b


def is_black(r, g, b, threshold=60):
    """คืนค่า True ถ้าสีที่อ่านได้มืดพอจะถือว่าเป็นเส้นดำ"""
    return r < threshold and g < threshold and b < threshold


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

        for index, topic in enumerate(TOPICS):
            node.create_subscription(
                Image, topic, self._make_callback(index), 10)

    def _make_callback(self, index):
        def callback(msg):
            r, g, b = image_to_rgb(msg)
            self.black[index] = is_black(r, g, b, self.threshold)
        return callback

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
