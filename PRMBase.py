import random
import math
import matplotlib.pyplot as plt


class PRM3D:
    def __init__(self, start, goal, num_nodes, connection_radius, workspace_bounds, obstacles=None):
        self.start = start
        self.goal = goal
        self.num_nodes = num_nodes
        self.connection_radius = connection_radius
        self.workspace_bounds = workspace_bounds
        self.obstacles = obstacles or []
        self.nodes = []
        self.edges = []
        self.path = []

    def generate_random_config(self):
        return [
            random.uniform(bound[0], bound[1]) for bound in self.workspace_bounds
        ]

    def is_collision_free(self, config1, config2):
        # Check for collision with obstacles
        for obstacle in self.obstacles:
            if (
                config1[0] < obstacle[1][0] and config2[0] > obstacle[0][0] and
                config1[1] < obstacle[1][1] and config2[1] > obstacle[0][1] and
                config1[2] < obstacle[1][2] and config2[2] > obstacle[0][2]
            ):
                return False

        return True

    def distance(self, config1, config2):
        return math.sqrt(sum((c1 - c2)**2 for c1, c2 in zip(config1, config2)))

    def generate_prm(self):
        self.nodes = [self.start, self.goal]  # Include start and goal in nodes
        for _ in range(self.num_nodes - 2):
            config = self.generate_random_config()
            self.nodes.append(config)

        for i in range(self.num_nodes):
            for j in range(i + 1, self.num_nodes):
                if (
                    self.distance(self.nodes[i], self.nodes[j]) < self.connection_radius and
                    self.is_collision_free(self.nodes[i], self.nodes[j])
                ):
                    self.edges.append((i, j))

    def find_path_dfs(self):
        start_node = self.nodes.index(self.start)
        goal_node = self.nodes.index(self.goal)
        visited = set()
        path = []
        self._dfs_path(start_node, goal_node, visited, path)
        return path

    def _dfs_path(self, current_node, goal, visited, path):
        visited.add(current_node)
        path.append(current_node)

        if current_node == goal:
            # Found a complete path
            return

        neighbors = [n[1] for n in self.edges if n[0] == current_node]
        for neighbor in neighbors:
            if neighbor not in visited:
                self._dfs_path(neighbor, goal, visited, path)

    def plot_prm(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot nodes
        for node in self.nodes:
            ax.scatter(node[0], node[1], node[2], c='b' if node != self.goal else 'r', marker='o')

        # Plot edges
        for edge in self.edges:
            node1 = self.nodes[edge[0]]
            node2 = self.nodes[edge[1]]
            ax.plot([node1[0], node2[0]], [node1[1], node2[1]], [node1[2], node2[2]], c='g')

        # Plot obstacles
        for obstacle in self.obstacles:
            ax.scatter(
                [obstacle[0][0], obstacle[1][0], obstacle[0][0], obstacle[1][0]],
                [obstacle[0][1], obstacle[1][1], obstacle[0][1], obstacle[1][1]],
                [obstacle[0][2], obstacle[0][2], obstacle[1][2], obstacle[1][2]],
                c='k', marker='x'
            )

        # Plot path
        if self.path:
            path_nodes = [self.nodes[node] for node in self.path]
            ax.plot(
                [node[0] for node in path_nodes],
                [node[1] for node in path_nodes],
                [node[2] for node in path_nodes],
                c='b'
            )

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

# Example usage with obstacles
start_point = [1, 1, 1]
goal_point = [9, 9, 9]
num_nodes = 100
connection_radius = 3.0
workspace_bounds = [(0, 10), (0, 10), (0, 10)]

# Define obstacles as tuples (lower bound, upper bound)
obstacles = [([2, 2, 2], [4, 4, 4]), ([6, 6, 6], [8, 8, 8])]

prm = PRM3D(start_point, goal_point, num_nodes, connection_radius, workspace_bounds, obstacles)
prm.generate_prm()
path = prm.find_path_dfs()
prm.plot_prm()
