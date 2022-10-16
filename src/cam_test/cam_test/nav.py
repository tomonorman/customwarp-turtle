import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class Nav(Node):

    def __init__(self):
        super().__init__('nav')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        move = Twist()
        move.linear.x=0.23
        move.angular.z=0.0
        self.publisher_.publish(move)


def main(args=None):
    rclpy.init(args=args)

    nav = Nav()

    rclpy.spin(nav)

    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()