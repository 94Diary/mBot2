import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

SEQUENCE = [
    ('forward',0.15,0.0,5.0),
    ('backward',-0.15,0.0,3.0),
    ("turnleft",0.0,  0.6, 2.0),
    ('forward', 0.15,  0.0, 3.0),
    ('turnright',0.0, -0.6, 2.0)
]


class MazeSolver(Node):

    def __init__(self):
        super().__init__('maze_solver')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel',10)
        self.timer = self.create_timer(0.1,self.tick)
        
        self.step_index = 0
        self.step_start_time = time.time()
        self.finished = False

        self.get_logger().info(f'start No.1 {SEQUENCE[0][0]}')

    def tick(self):
        if self.finished:
            return
        
        name,linear ,angular,duration = SEQUENCE[self.step_index]
        elapsed = time.time() - self.step_start_time

        if elapsed >= duration:
            self.step_index += 1
            self.step_start_time = time.time()

            if self.step_index >= len(SEQUENCE):
                self.cmd_pub.publish(Twist())
                self.get_logger().info("ครบแล้ว")
                self.finished = True
                return

        name,linear,angular,duration = SEQUENCE[self.step_index]
        self.get_logger().info(f'No1, {self.step_index + 1 }: {name}')

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MazeSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()