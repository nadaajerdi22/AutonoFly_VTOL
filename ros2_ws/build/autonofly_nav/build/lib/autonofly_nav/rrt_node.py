import rclpy
from rclpy.node import Node
from autonofly_nav.rrt_planner import RRT  
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, TimesyncStatus
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_node')
        self.get_logger().info("RRT Node démarré")

        # Timestamp PX4 (obligatoire)
        self.timestamp = 0
        self.timesync_sub = self.create_subscription(
            TimesyncStatus,
            '/fmu/out/timesync_status',
            self.timesync_callback,
            10
        )

        # PX4 publishers
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        # Visualisation RViz (optionnel)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.rviz_path = Path()
        self.rviz_path.header.frame_id = 'map'

        # RRT planning
        self.planner = RRT(
            start=[0, 0],
            goal=[6, 10],
            obstacles=[(5, 5, 1), (3, 6, 2), (3, 8, 2)],
            rand_area=[-2, 15],
            robot_radius=0.8
        )
        path = self.planner.planning(animation=False)

        if path:
            self.get_logger().info(f"Chemin trouvé avec {len(path)} points")
            self.path = path[::-1]  # Reverse path: start -> goal
            self.current_idx = 0

            # Visualisation du chemin dans RViz
            for x, y in self.path:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.position.z = -2.0
                self.rviz_path.poses.append(pose)
            self.path_pub.publish(self.rviz_path)

            # Timer pour envoyer en boucle un setpoint (obligatoire pour OFFBOARD)
            self.hold_timer = self.create_timer(0.1, self.publish_position_hold)

            # Armer et activer OFFBOARD après un petit délai
            self.create_timer(2.0, self.arm_and_set_offboard)

            # Timer pour envoyer les waypoints RRT
            self.timer = self.create_timer(3.0, self.publish_next_waypoint)
        else:
            self.get_logger().warn("Aucun chemin trouvé")

    def timesync_callback(self, msg):
        self.timestamp = msg.timestamp

    def arm_and_set_offboard(self):
        self.get_logger().info("Envoi des commandes pour mode OFFBOARD + ARM")

        # Mode OFFBOARD
        offboard_cmd = VehicleCommand()
        offboard_cmd.timestamp = self.timestamp
        offboard_cmd.param1 = 1.0  # Start OFFBOARD
        offboard_cmd.command = 176
        offboard_cmd.target_system = 1
        offboard_cmd.target_component = 1
        offboard_cmd.source_system = 1
        offboard_cmd.source_component = 1
        self.cmd_pub.publish(offboard_cmd)

        # Armement
        arm_cmd = VehicleCommand()
        arm_cmd.timestamp = self.timestamp
        arm_cmd.param1 = 1.0  # Arm
        arm_cmd.command = 400
        arm_cmd.target_system = 1
        arm_cmd.target_component = 1
        arm_cmd.source_system = 1
        arm_cmd.source_component = 1
        self.cmd_pub.publish(arm_cmd)

    def publish_position_hold(self):
        # Publie constamment un point (obligatoire pour rester en OFFBOARD)
        sp = TrajectorySetpoint()
        sp.timestamp = self.timestamp
        sp.position = [0.0, 0.0, -2.0]
        sp.yaw = 0.0
        self.setpoint_pub.publish(sp)

    def publish_next_waypoint(self):
        if self.current_idx >= len(self.path):
            self.get_logger().info("Tous les waypoints ont été publiés")
            return

        x, y = self.path[self.current_idx]
        sp = TrajectorySetpoint()
        sp.timestamp = self.timestamp
        sp.position = [float(x), float(y), -2.0]
        sp.yaw = 0.0
        self.setpoint_pub.publish(sp)

        self.get_logger().info(f"Waypoint {self.current_idx+1}/{len(self.path)} envoyé : {sp.position}")
        self.current_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = RRTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
