# offboard_control.py
import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint

def main():
    rclpy.init()
    node = rclpy.create_node('offboard_controller')
    pub_mode = node.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
    pub_setpoint = node.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

    # Exemple : décoller à 5m et avancer
    msg_mode = OffboardControlMode()
    msg_mode.position = True
    msg_mode.velocity = False

    msg_sp = TrajectorySetpoint()
    msg_sp.x, msg_sp.y, msg_sp.z = 0.0, 0.0, -5.0  # NED frame (Z vers le bas)

    while rclpy.ok():
        pub_mode.publish(msg_mode)
        pub_setpoint.publish(msg_sp)
        node.get_logger().info("Envoi des commandes OFFBOARD...")
        rclpy.spin_once(node, timeout_sec=0.1)