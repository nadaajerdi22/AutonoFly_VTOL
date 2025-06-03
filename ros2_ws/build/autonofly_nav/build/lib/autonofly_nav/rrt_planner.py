# ce code contient la classe RRT pour le planificateur de chemin RRT
import random
import math
import matplotlib.pyplot as plt
import numpy as np

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.path_x = []
        self.path_y = []
        self.parent = None

class RRT:
    def __init__(self, start, goal, obstacles, rand_area, expand_dist=3.0,path_resolution=0.5, goal_sample_rate=5, max_iter=500,robot_radius=0.0, play_area=None):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.obstacles = obstacles
        self.min_rand, self.max_rand = rand_area
        self.expand_dist = expand_dist
        self.path_res = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.robot_radius = robot_radius
        self.play_area = play_area  # [xmin, xmax, ymin, ymax] or None
        self.node_list = []

    def planning(self, animation=True):
        self.node_list = [self.start]

        for i in range(self.max_iter):
            rnd = self.get_random_node()
            nearest_idx = self.get_nearest_node_index(rnd)
            nearest_node = self.node_list[nearest_idx]

            new_node = self.steer(nearest_node, rnd)

            if not self.is_in_play_area(new_node):
                continue
            if not self.collision_free(new_node):
                continue

            self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd)

            if self.distance_to_goal(new_node) <= self.expand_dist:
                final_node = self.steer(new_node, self.goal)
                if self.collision_free(final_node):
                    return self.generate_final_path(len(self.node_list) - 1)

        return None

    def steer(self, from_node, to_node):
        new_node = Node(from_node.x, from_node.y)
        dist, theta = self.calc_dist_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        extend_len = min(self.expand_dist, dist)
        n_expand = int(extend_len / self.path_res)

        for _ in range(n_expand):
            new_node.x += self.path_res * math.cos(theta)
            new_node.y += self.path_res * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        # snap to goal if close enough
        dist, _ = self.calc_dist_angle(new_node, to_node)
        if dist <= self.path_res:
            new_node.x = to_node.x
            new_node.y = to_node.y
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)

        new_node.parent = from_node
        return new_node

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            x = random.uniform(self.min_rand, self.max_rand)
            y = random.uniform(self.min_rand, self.max_rand)
            return Node(x, y)
        else:
            return Node(self.goal.x, self.goal.y)

    def get_nearest_node_index(self, rnd_node):
        distances = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 for node in self.node_list]
        return distances.index(min(distances))

    def collision_free(self, node):
        for (ox, oy, size) in self.obstacles:
            for (x, y) in zip(node.path_x, node.path_y):
                dx = ox - x
                dy = oy - y
                if dx*dx + dy*dy <= (size + self.robot_radius)**2:
                    return False
        return True

    def is_in_play_area(self, node):
        if self.play_area is None:
            return True
        xmin, xmax, ymin, ymax = self.play_area
        return xmin <= node.x <= xmax and ymin <= node.y <= ymax

    def distance_to_goal(self, node):
        dx = node.x - self.goal.x
        dy = node.y - self.goal.y
        return math.hypot(dx, dy)

    def calc_dist_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def generate_final_path(self, goal_index):
        path = [[self.goal.x, self.goal.y]]
        node = self.node_list[goal_index]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([self.start.x, self.start.y])
        return path[::-1]

    def draw_graph(self, rnd=None):
        plt.clf()
        if rnd:
            plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")
        for (ox, oy, size) in self.obstacles:
            self.plot_circle(ox, oy, size)
        if self.play_area:
            xmin, xmax, ymin, ymax = self.play_area
            plt.plot([xmin, xmax, xmax, xmin, xmin],[ymin, ymin, ymax, ymax, ymin], "-k")
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis("equal")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):
        degs = np.deg2rad(np.arange(0, 361, 5))
        plt.plot(x + size * np.cos(degs), y + size * np.sin(degs), color)

def main():
    obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2),
                    (7, 5, 2), (9, 5, 2), (8, 10, 1)]
    rrt = RRT(start=[0, 0], goal=[6, 10], obstacles=obstacleList,
              rand_area=[-2, 15], robot_radius=0.8)
    path = rrt.planning(animation=True)
    if path is None:
        print("No path found")
    else:
        print("Path found!")
        rrt.draw_graph()
        plt.plot([x for x, y in path], [y for x, y in path], '-r')
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    main()
