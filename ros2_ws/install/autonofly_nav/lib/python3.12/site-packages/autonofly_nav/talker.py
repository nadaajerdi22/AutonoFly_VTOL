import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = Node('takler_node')
    node.get_logger().info('Takler node lancé!')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
