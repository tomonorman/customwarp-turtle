import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = Node('auto_stop')
    rclpy.spin_once(node, timeout_sec=23)
    node.get_logger().debug("Stopping simulation...")
    rclpy.shutdown()
if __name__ == '__main__':
    main()