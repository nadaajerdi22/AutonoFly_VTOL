import rclpy
from rclpy.node import Node
from autonofly_nav.rrt_planner import RRTPlanner  # importe ta classe RRTPlanner

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_node')
        self.get_logger().info("RRT Node démarré")
        self.planner = RRTPlanner()
        # Tu peux ici ajouter publishers, subscribers, timers, etc.

def main(args=None):
    rclpy.init(args=args)
    node = RRTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()