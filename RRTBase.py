import random
import math
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2 + (node1.z - node2.z)**2)

def generate_random_node(bounds):
    x = random.uniform(bounds[0], bounds[1])
    y = random.uniform(bounds[2], bounds[3])
    z = random.uniform(bounds[4], bounds[5])
    return Node(x, y, z)

def nearest_neighbor(tree, random_node):
    min_distance = float('inf')
    nearest_node = None
    for node in tree:
        d = distance(node, random_node)
        if d < min_distance:
            min_distance = d
            nearest_node = node
    return nearest_node

def steer(from_node, to_node, max_distance):
    d = distance(from_node, to_node)
    if d < max_distance:
        return to_node
    else:
        ratio = max_distance / d
        x = from_node.x + ratio * (to_node.x - from_node.x)
        y = from_node.y + ratio * (to_node.y - from_node.y)
        z = from_node.z + ratio * (to_node.z - from_node.z)
        return Node(x, y, z)

def is_collision_free(from_node, to_node, obstacles):
    # Assuming obstacles is a list of rectangular prisms represented by their min and max coordinates
    for obstacle in obstacles:
        if (from_node.x < obstacle[1][0] and to_node.x > obstacle[0][0] and
            from_node.y < obstacle[1][1] and to_node.y > obstacle[0][1] and
            from_node.z < obstacle[1][2] and to_node.z > obstacle[0][2]):
            return False
    return True

def rrt(start, goal, bounds, max_distance, num_nodes, obstacles):
    tree = [start]

    for _ in range(num_nodes):
        random_node = generate_random_node(bounds)
        nearest_node = nearest_neighbor(tree, random_node)
        new_node = steer(nearest_node, random_node, max_distance)

        if is_collision_free(nearest_node, new_node, obstacles):
            new_node.parent = nearest_node
            tree.append(new_node)

        if distance(new_node, goal) < max_distance:
            goal.parent = new_node
            tree.append(goal)
            break

    return tree

def plot_3d_tree(tree, goal, obstacles):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for node in tree:
        if node.parent:
            ax.plot([node.parent.x, node.x], [node.parent.y, node.y], [node.parent.z, node.z], color='blue')

    if goal.parent:
        path_x = [goal.x]
        path_y = [goal.y]
        path_z = [goal.z]
        while goal.parent:
            path_x.append(goal.parent.x)
            path_y.append(goal.parent.y)
            path_z.append(goal.parent.z)
            goal = goal.parent
        ax.plot(path_x, path_y, path_z, color='green', linewidth=2)

    for obstacle in obstacles:
        ax.scatter(*zip(*obstacle), color='red', marker='s')

    ax.scatter(goal.x, goal.y, goal.z, color='green', marker='o')
    ax.scatter(tree[0].x, tree[0].y, tree[0].z, color='orange', marker='o')  # start node
    plt.show()

if __name__ == "__main__":
    start_node = Node(0, 0, 0)
    goal_node = Node(10, 10, 10)
    space_bounds = [-2, 12, -2, 12, -2, 12]
    max_distance = 1.0
    num_nodes = 500
    obstacles = [((1, 7), (1, 7), (1, 7)), ((6, 8), (6, 8), (6, 8))]  # Example obstacles

    tree = rrt(start_node, goal_node, space_bounds, max_distance, num_nodes, obstacles)
    plot_3d_tree(tree, goal_node, obstacles)
