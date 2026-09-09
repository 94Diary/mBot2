"""อ่านมุมหมุนล้อซ้าย-ขวาจาก /joint_states (เทียบเท่า encoder ของจริง)"""

from sensor_msgs.msg import JointState


class WheelEncoder:
    """สมัคร subscriber ฟัง /joint_states แล้วเก็บมุมหมุน (เรเดียน) ของล้อแต่ละข้างไว้

    ยังไม่ได้ใช้ในการตัดสินใจเดินตามเส้นตอนนี้ แต่เตรียมไว้ใช้ต่อยอด
    (เช่น คำนวณระยะทางที่วิ่งไปแล้ว หรือใช้ตอนทำ Trémaux's algorithm)

    ใช้แบบนี้:
        self.encoder = WheelEncoder(self)   # self คือ Node
        ...
        print(self.encoder.left_position, self.encoder.right_position)
    """

    def __init__(self, node):
        self.left_position = 0.0
        self.right_position = 0.0
        node.create_subscription(JointState, '/joint_states', self._callback, 10)

    def _callback(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            if name == 'left_wheel_joint':
                self.left_position = position
            elif name == 'right_wheel_joint':
                self.right_position = position
