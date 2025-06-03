import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

class Landmark:
    def __init__(self, mu, sigma):
        self.mu = mu
        self.sigma = sigma

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
        self.trajectory = []

    def motion_update(self, control, dt):
        v, w = control
        for p in self.particles:
            p.theta += w * dt + np.random.randn() * 0.005
            p.theta = (p.theta + np.pi) % (2 * np.pi) - np.pi
            p.x += v * dt * np.cos(p.theta) + np.random.randn() * 0.05
            p.y += v * dt * np.sin(p.theta) + np.random.randn() * 0.05

    def sensor_update(self, observations, sensor_noise=0.5):
        R = np.eye(2) * sensor_noise  # bruit observation
        for p in self.particles:
            for obs_id, obs_range, obs_bearing in observations:
                # Calcul position observée en coordonnées globales
                lx = p.x + obs_range * np.cos(p.theta + obs_bearing)
                ly = p.y + obs_range * np.sin(p.theta + obs_bearing)
                z = np.array([lx, ly])

                if obs_id not in p.landmarks:
                    # Initialisation du landmark
                    p.landmarks[obs_id] = Landmark(z, np.eye(2) * 1.0)
                else:
                    landmark = p.landmarks[obs_id]
                    mu = landmark.mu
                    sigma = landmark.sigma

                    # Innovation
                    y = z - mu

                    # Jacobienne H = identité (car mesure directe position landmark)
                    H = np.eye(2)

                    # Covariance innovation
                    S = H @ sigma @ H.T + R

                    # Gain de Kalman
                    K = sigma @ H.T @ np.linalg.inv(S)

                    # Mise à jour moyenne et covariance
                    landmark.mu = mu + K @ y
                    landmark.sigma = (np.eye(2) - K @ H) @ sigma

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
                    predicted_bearing = (predicted_bearing + np.pi) % (2 * np.pi) - np.pi

                    range_error = obs_range - predicted_range
                    bearing_error = obs_bearing - predicted_bearing
                    bearing_error = (bearing_error + np.pi) % (2 * np.pi) - np.pi

                    range_var = 0.5 ** 2
                    bearing_var = (np.deg2rad(10)) ** 2

                    p_range = np.exp(-0.5 * (range_error ** 2) / range_var) / np.sqrt(2 * np.pi * range_var)
                    p_bearing = np.exp(-0.5 * (bearing_error ** 2) / bearing_var) / np.sqrt(2 * np.pi * bearing_var)

                    weight *= p_range * p_bearing
            p.weight = weight
            total_weight += weight

        EPS = 1e-10
        if total_weight < EPS:
            total_weight = EPS  # éviter division par zéro

        for p in self.particles:
            p.weight /= total_weight

    def resample(self):
        weights = np.array([p.weight for p in self.particles])
        weights_sum = np.sum(weights)
        EPS = 1e-10
        if weights_sum < EPS:
            weights_sum = EPS
        weights /= weights_sum

        positions = (np.arange(self.num_particles) + np.random.uniform()) / self.num_particles
        cumulative_sum = np.cumsum(weights)
        cumulative_sum[-1] = 1.0  # éviter erreurs numériques

        indexes = np.searchsorted(cumulative_sum, positions)
        self.particles = [self.particles[i] for i in indexes]
        for p in self.particles:
            p.weight = 1.0 / self.num_particles

    def get_best_estimate(self):
        best = max(self.particles, key=lambda p: p.weight)
        self.trajectory.append((best.x, best.y))
        return best.x, best.y, best.theta, best.landmarks

class FastSLAMNode(Node):
    def __init__(self):
        super().__init__('fastslam_node')
        self.fastslam = FastSLAM2(num_particles=100, map_size=100)
        self.timer = self.create_timer(0.1, self.timer_callback)

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.est_traj_plot, = self.ax.plot([], [], 'b-', label="Estimation Traj")
        self.lm_plot, = self.ax.plot([], [], 'ro', label="Landmarks")
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 100)
        self.ax.legend()

    def timer_callback(self):
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

        traj_x, traj_y = zip(*self.fastslam.trajectory)
        self.est_traj_plot.set_data(traj_x, traj_y)
        if est_landmarks:
            lm_x = [lm.mu[0] for lm in est_landmarks.values()]
            lm_y = [lm.mu[1] for lm in est_landmarks.values()]
            self.lm_plot.set_data(lm_x, lm_y)
        self.ax.figure.canvas.draw()
        self.ax.figure.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = FastSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    main()
