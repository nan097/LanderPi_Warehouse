import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = Node('benchmark_runner')
    node.get_logger().warning('NOT_IMPLEMENTED: backend integration pending')
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
