import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

class Landmark:
    def __init__(self, mu, sigma):
        self.mu = mu  # Mean position (x, y)
        self.sigma = sigma  # Covariance matrix

class Particle:
    def __init__(self, x, y, theta, weight):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight
        self.landmarks = {}

class FastSLAM2:
    def __init__(self, num_particles, map_size):
        self.num_particles = num_particles
        self.map_size = map_size
        self.particles = [
            Particle(
                np.random.uniform(0, map_size),
                np.random.uniform(0, map_size),
                np.random.uniform(-np.pi, np.pi),
                1.0 / num_particles
            )
            for _ in range(num_particles)
        ]

    def motion_update(self, control, dt):
        v, w = control
        for p in self.particles:
            p.theta += w * dt + np.random.randn() * 0.01  # Ajout de bruit
            p.theta = (p.theta + np.pi) % (2 * np.pi) - np.pi
            p.x += v * dt * np.cos(p.theta) + np.random.randn() * 0.05
            p.y += v * dt * np.sin(p.theta) + np.random.randn() * 0.05

    def sensor_update(self, observations, sensor_noise=0.5):
        for p in self.particles:
            for obs_id, obs_range, obs_bearing in observations:
                lx = p.x + obs_range * np.cos(p.theta + obs_bearing)
                ly = p.y + obs_range * np.sin(p.theta + obs_bearing)
                if obs_id not in p.landmarks:
                    # Initialiser landmark avec une incertitude élevée
                    p.landmarks[obs_id] = Landmark(np.array([lx, ly]), np.eye(2) * 1.0)
                else:
                    landmark = p.landmarks[obs_id]
                    z_pred = landmark.mu
                    Q = landmark.sigma + np.eye(2) * sensor_noise
                    K = Q @ np.linalg.inv(Q + np.eye(2) * sensor_noise)
                    z = np.array([lx, ly])
                    landmark.mu = z_pred + K @ (z - z_pred)
                    landmark.sigma = (np.eye(2) - K) @ Q

    def compute_weights(self, observations):
        total_weight = 0.0
        for p in self.particles:
            weight = 1.0
            for obs_id, obs_range, obs_bearing in observations:
                if obs_id in p.landmarks:
                    landmark = p.landmarks[obs_id]
                    dx = landmark.mu[0] - p.x
                    dy = landmark.mu[1] - p.y
                    predicted_range = np.hypot(dx, dy)
                    predicted_bearing = np.arctan2(dy, dx) - p.theta
                    range_error = obs_range - predicted_range
                    bearing_error = ((obs_bearing - predicted_bearing) + np.pi) % (2*np.pi) - np.pi
                    weight *= np.exp(-0.5 * (range_error**2 + bearing_error**2))
            p.weight = weight
            total_weight += weight
        # Normalisation
        if total_weight == 0:
            for p in self.particles:
                p.weight = 1.0 / self.num_particles
        else:
            for p in self.particles:
                p.weight /= total_weight

    def resample(self):
        weights = [p.weight for p in self.particles]
        weight_sum = sum(weights)
        if weight_sum == 0:
            weights = [1.0/self.num_particles] * self.num_particles
        else:
            weights = np.array(weights) / weight_sum
            weights /= np.sum(weights)  # Normalisation stricte
            indices = np.random.choice(range(self.num_particles), self.num_particles, p=weights)
            self.particles = [self.particles[i] for i in indices]

    def get_best_estimate(self):
        best = max(self.particles, key=lambda p: p.weight)
        return best.x, best.y, best.theta, best.landmarks

class FastSLAMNode(Node):
    def __init__(self):
        super().__init__('fastslam_node')
        self.fastslam = FastSLAM2(num_particles=100, map_size=100)
        self.timer = self.create_timer(0.1, self.timer_callback)  # appel toutes les 0.1 sec

    def timer_callback(self):
        # Simuler contrôle et observations
        control = (1.0, 0.1)
        observations = [
            (1, 20 + np.random.randn(), np.pi / 6 + np.random.randn() * 0.1),
            (2, 25 + np.random.randn(), -np.pi / 4 + np.random.randn() * 0.1)
        ]
        self.fastslam.motion_update(control, 0.1)
        self.fastslam.sensor_update(observations)
        self.fastslam.compute_weights(observations)
        self.fastslam.resample()
        est_x, est_y, est_theta, est_landmarks = self.fastslam.get_best_estimate()
        self.get_logger().info(f"Estimate: x={est_x:.2f}, y={est_y:.2f}, theta={est_theta:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = FastSLAMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
