import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from fastslam import FastSLAM2  

class SlamSubscriberNode(Node):
    def __init__(self):
        super().__init__('slam_subscriber_node')
        self.fastslam = FastSLAM2(num_particles=100, map_size=100)

        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.control = (0.0, 0.0)
        self.observations = []

        self.timer = self.create_timer(0.1, self.timer_callback)

    def lidar_callback(self, msg):
        self.observations = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        for i, r in enumerate(msg.ranges):
            if np.isfinite(r) and r < msg.range_max:
                bearing = angle_min + i * angle_increment
                obs_id = i
                self.observations.append((obs_id, r, bearing))

    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        self.control = (vx, wz)

    def timer_callback(self):
        self.fastslam.motion_update(self.control, 0.1)
        self.fastslam.sensor_update(self.observations)
        self.fastslam.compute_weights(self.observations)
        self.fastslam.resample()
        est_x, est_y, est_theta, est_landmarks = self.fastslam.get_best_estimate()
        self.get_logger().info(f"Estimate: x={est_x:.2f}, y={est_y:.2f}, theta={est_theta:.2f}")
        # Ici tu peux aussi faire de l'affichage si tu veux, par exemple avec matplotlib ou RViz

def main(args=None):
    rclpy.init(args=args)
    node = SlamSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
