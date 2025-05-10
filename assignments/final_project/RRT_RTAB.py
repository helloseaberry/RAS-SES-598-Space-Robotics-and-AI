import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import random
import math

# RRT Structures
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def distance(p1, p2):
    return math.hypot(p1.x - p2.x, p1.y - p2.y)

def is_collision(p1, p2, occ_grid):
    x1, y1 = int(p1.x), int(p1.y)
    x2, y2 = int(p2.x), int(p2.y)
    for x, y in bresenham(x1, y1, x2, y2):
        if 0 <= x < occ_grid.shape[1] and 0 <= y < occ_grid.shape[0]:
            if occ_grid[y, x] == 0: 
                return True
    return False

def bresenham(x0, y0, x1, y1):
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = 1 if x0 < x1 else -1, 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy: err -= dy; x0 += sx
        if e2 < dx: err += dx; y0 += sy

def sample_free(grid, goal, goal_bias=0.1):
    h, w = grid.shape
    if random.random() < goal_bias:
        return Node(goal.x, goal.y)
    while True:
        x, y = random.randint(0, w - 1), random.randint(0, h - 1)
        if grid[y, x] == 255: 
            return Node(x, y)

def nearest(tree, point):
    return min(tree, key=lambda n: distance(n, point))

def steer(from_node, to_node, step=5):
    theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
    new_x = int(from_node.x + step * math.cos(theta))
    new_y = int(from_node.y + step * math.sin(theta))
    new_node = Node(new_x, new_y)
    new_node.parent = from_node
    return new_node

def build_rrt(start, goal, grid, max_iter=5000, step=5, thresh=10, goal_bias=0.1):
    tree = [start]
    for _ in range(max_iter):
        rnd = sample_free(grid, goal, goal_bias)
        nearest_node = nearest(tree, rnd)
        new_node = steer(nearest_node, rnd, step)

        if 0 <= new_node.x < grid.shape[1] and 0 <= new_node.y < grid.shape[0]:
            if grid[new_node.y, new_node.x] == 255 and not is_collision(nearest_node, new_node, grid):
                tree.append(new_node)
                if distance(new_node, goal) < thresh:
                    goal.parent = new_node
                    return tree, goal
    return tree, None

def extract_path(goal_node):
    path = []
    node = goal_node
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

# Load Point Cloud and Convert to Grid
pcd = o3d.io.read_point_cloud("cloud.ply")
points = np.asarray(pcd.points)

if points.shape[0] == 0:
    print("\u274c Point cloud is empty!")
    exit()

xy = points[:, :2]
res = 0.05
min_xy = xy.min(axis=0)
dims = ((xy.max(axis=0) - min_xy) / res).astype(int) + 1

# Free space is 0
grid = np.full((dims[1], dims[0]), 0, dtype=np.uint8) 

# Obstacles are 255
for x, y in xy:
    i = int((x - min_xy[0]) / res)
    j = int((y - min_xy[1]) / res)
    if 0 <= i < dims[0] and 0 <= j < dims[1]:
        grid[j, i] = 255 


clicks = []

def onclick(event):
    if event.xdata is not None and event.ydata is not None:
        x, y = int(event.xdata), int(event.ydata)
        if 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]:
            val = grid[y, x]
            print(f"Clicked ({x}, {y}) - Grid value: {val}")
            if val == 255:
                clicks.append((x, y))
                plt.plot(x, y, 'go' if len(clicks) == 1 else 'bo')
                plt.draw()
                if len(clicks) == 2:
                    plt.close()
            else:
                print(f"Click in a white area (grid value 255).")

plt.imshow(grid, cmap='gray')  
plt.title("Select start and goal node")
plt.connect('button_press_event', onclick)
plt.grid(True)
plt.show()


start = Node(*clicks[0])
goal = Node(*clicks[1])
print(f"Start: ({start.x}, {start.y}), Goal: ({goal.x}, {goal.y})")

# RRT Planning
tree, goal_node = build_rrt(start, goal, grid)

# Visualize Result
plt.imshow(grid, cmap='gray')
for node in tree:
    if node.parent:
        plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color='blue', linewidth=0.5)

if goal_node:
    path = extract_path(goal_node)
    px, py = zip(*path)
    plt.plot(px, py, '-r', label='RRT Path')
else:
    print("Path not found.")

plt.plot(start.x, start.y, 'go', label='Start')
plt.plot(goal.x, goal.y, 'bo', label='Goal')
plt.legend()
plt.title('RRT Path on Occupancy Grid')
plt.grid(True)
plt.show()
