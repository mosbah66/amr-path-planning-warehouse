import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
import heapq

def heuristic(a, b):
    return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5

def a_star(start, goal, obstacles, grid_size, bounds):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    cost_so_far = {start: 0}
    directions = [(-1,0),(1,0),(0,-1),(0,1)]

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            break
        for dx, dy in directions:
            neighbor = (current[0]+dx, current[1]+dy)
            if not (0 <= neighbor[0] <= bounds[0] and 0 <= neighbor[1] <= bounds[1]):
                continue
            point = Point(neighbor[0], neighbor[1])
            if any(polygon.contains(point) for polygon in obstacles):
                continue
            new_cost = cost_so_far[current] + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(goal, neighbor)
                heapq.heappush(open_set, (priority, neighbor))
                came_from[neighbor] = current

    path = []
    if goal in came_from:
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
    return path

# Example environment
obstacles = [Polygon([(4,4),(6,4),(6,6),(4,6)]), Polygon([(8,2),(10,2),(10,5),(8,5)])]
start, goal = (1,1), (12,8)
path = a_star(start, goal, obstacles, grid_size=1, bounds=(15, 10))

# Plot
fig, ax = plt.subplots()
for obs in obstacles:
    x, y = obs.exterior.xy
    ax.fill(x, y, color='gray')
if path:
    x, y = zip(*path)
    ax.plot(x, y, 'b-o')
ax.plot(start[0], start[1], 'go', label='Start')
ax.plot(goal[0], goal[1], 'ro', label='Goal')
ax.set_aspect('equal')
ax.grid(True)
ax.legend()
plt.show()
