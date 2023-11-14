import heapq
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Node:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.g = float('inf')
        self.h = 0
        self.parent = None

    def __lt__(self, other):
        return (self.g + self.h) < (other.g + other.h)

def heuristic(node, goal):
    return abs(node.x - goal.x) + abs(node.y - goal.y) + abs(node.z - goal.z)

def get_neighbors(node, grid):
    neighbors = []
    for i in range(-1, 2):
        for j in range(-1, 2):
            for k in range(-1, 2):
                if i == 0 and j == 0 and k == 0:
                    continue
                new_x, new_y, new_z = node.x + i, node.y + j, node.z + k
                if 0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]) and 0 <= new_z < len(grid[0][0]) and not grid[new_x][new_y][new_z]:
                    neighbors.append(Node(new_x, new_y, new_z))
    return neighbors

def astar_3d(grid, start, goal):
    open_set = []
    closed_set = set()

    start_node = Node(*start)
    goal_node = Node(*goal)

    start_node.g = 0
    start_node.h = heuristic(start_node, goal_node)

    heapq.heappush(open_set, start_node)

    while open_set:
        current_node = heapq.heappop(open_set)

        if current_node.x == goal_node.x and current_node.y == goal_node.y and current_node.z == goal_node.z:
            path = []
            while current_node:
                path.append((current_node.x, current_node.y, current_node.z))
                current_node = current_node.parent
            return path[::-1]

        closed_set.add((current_node.x, current_node.y, current_node.z))

        for neighbor in get_neighbors(current_node, grid):
            if (neighbor.x, neighbor.y, neighbor.z) in closed_set:
                continue

            tentative_g = current_node.g + 1

            if tentative_g < neighbor.g:
                neighbor.parent = current_node
                neighbor.g = tentative_g
                neighbor.h = heuristic(neighbor, goal_node)

                if neighbor not in open_set:
                    heapq.heappush(open_set, neighbor)

    return None

# Example usage:
grid = [[[0, 0, 0], [0, 1, 0], [0, 0, 0]],
        [[0, 0, 0], [0, 1, 0], [0, 0, 0]],
        [[0, 0, 0], [0, 0, 0], [0, 0, 0]]]

start_point = (0, 0, 0)
end_point = (2, 2, 2)

path = astar_3d(grid, start_point, end_point)

# Visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot grid
for i in range(len(grid)):
    for j in range(len(grid[0])):
        for k in range(len(grid[0][0])):
            if grid[i][j][k] == 1:
                ax.scatter(i, j, k, color='black', marker='o')
            else:
                ax.scatter(i, j, k, color='red', marker='o')

# Plot path
if path:
    path_x, path_y, path_z = zip(*path)
    ax.plot(path_x, path_y, path_z, color='green', marker='o')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
