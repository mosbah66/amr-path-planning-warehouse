import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
import heapq

# Heuristic function for A*
def heuristic(a, b):
    return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5

# A* pathfinding function
def a_star(start, goal, obstacles, bounds):
    directions = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    cost_so_far = {start: 0}
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            break
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            point = Point(neighbor[0], neighbor[1])
            if not (0 <= neighbor[0] <= bounds[0] and 0 <= neighbor[1] <= bounds[1]):
                continue
            if any(p.contains(point) for p in obstacles):
                continue
            new_cost = cost_so_far[current] + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(goal, neighbor)
                heapq.heappush(open_set, (priority, neighbor))
                came_from[neighbor] = current
    # Reconstruct path
    path = []
    if goal in came_from:
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
    return path

# Plotting utility
def plot_environment(obstacles, path, start, goal, title, failed_path=None):
    fig, ax = plt.subplots()
    for poly in obstacles:
        x, y = poly.exterior.xy
        ax.fill(x, y, color='lightgray', alpha=0.7)
    if path:
        px, py = zip(*path)
        ax.plot(px, py, 'b-o', label="Path (A*)")
    if failed_path:
        fx, fy = zip(*failed_path)
        ax.plot(fx, fy, 'r--', label="Failed Path")
    ax.plot(start[0], start[1], 'go', label='Start')
    ax.plot(goal[0], goal[1], 'ro', label='Goal')
    ax.set_title(title)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend()
    plt.show()

# ----------------------------
# Figure 1: Simple Pathfinding
# ----------------------------
obstacles1 = [
    Polygon([(5,2),(7,2),(7,4),(5,4)]),
    Polygon([(2,6),(4,6),(4,8),(2,8)])
]
start1, goal1 = (1, 1), (10, 9)
path1 = a_star(start1, goal1, obstacles1, bounds=(12,12))
plot_environment(obstacles1, path1, start1, goal1, "Figure 1: Simple Pathfinding")

# --------------------------------------------
# Figure 2: Navigation Through Narrow Corridors
# --------------------------------------------
obstacles2 = [
    Polygon([(3,2),(4,2),(4,10),(3,10)]),
    Polygon([(6,2),(7,2),(7,10),(6,10)])
]
start2, goal2 = (1, 6), (9, 6)
path2 = a_star(start2, goal2, obstacles2, bounds=(12,12))
plot_environment(obstacles2, path2, start2, goal2, "Figure 2: Narrow Corridor Navigation")

# -----------------------------------------------------
# Figure 3: Dead End (Failed) and Alternate (Successful)
# -----------------------------------------------------
obstacles3 = [
    Polygon([(4,4),(8,4),(8,8),(4,8)]),  # Main block
    Polygon([(4,4),(5,4),(5,6),(4,6)])   # Small door left open
]
start3, goal3 = (2, 5), (6, 6)
failed_path3 = a_star(start3, (7, 5), [Polygon([(4,4),(8,4),(8,8),(4,8)])], bounds=(12,12))  # Trapped
path3 = a_star(start3, goal3, obstacles3, bounds=(12,12))  # Successful
plot_environment(obstacles3, path3, start3, goal3, "Figure 3: Trap Recovery", failed_path=failed_path3)
