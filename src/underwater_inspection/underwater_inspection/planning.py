"""Collision-free planar route for a known inspection site at fixed depth."""
import heapq
import math


def clearance(point, obstacles, robot_radius=0.0):
    return min((math.hypot(point[0] - o['x'], point[1] - o['y']) - o['radius'] - robot_radius
                for o in obstacles), default=float('inf'))


def plan_route(start, goal, cfg, resolution=0.25):
    xmin, xmax, ymin, ymax = cfg['bounds']
    inflated = cfg['robot_radius'] + cfg['planner_margin']

    def cell(point):
        return round((point[0] - xmin) / resolution), round((point[1] - ymin) / resolution)

    def position(c):
        return xmin + c[0] * resolution, ymin + c[1] * resolution

    def free(p):
        return (xmin <= p[0] <= xmax and ymin <= p[1] <= ymax
                and clearance(p, cfg['obstacles'], inflated) > 0)

    def visible(a, b):
        steps = max(1, math.ceil(math.dist(a, b) / (resolution / 2)))
        return all(free((a[0] + (b[0] - a[0]) * i / steps,
                         a[1] + (b[1] - a[1]) * i / steps)) for i in range(steps + 1))

    if not free(start) or not free(goal):
        raise ValueError('Start or goal lies outside the navigable site')
    first, last = cell(start), cell(goal)
    queue = [(0.0, first)]
    cost, parent = {first: 0.0}, {}
    while queue:
        _, current = heapq.heappop(queue)
        if current == last:
            break
        for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1),
                       (1, 1), (-1, 1), (1, -1), (-1, -1)):
            nxt = current[0] + dx, current[1] + dy
            a, b = position(current), position(nxt)
            if not visible(a, b):
                continue
            new_cost = cost[current] + math.hypot(dx, dy)
            if new_cost < cost.get(nxt, float('inf')):
                cost[nxt], parent[nxt] = new_cost, current
                heapq.heappush(queue, (new_cost + math.dist(nxt, last), nxt))
    else:
        raise ValueError('No collision-free route')
    path = [last]
    while path[-1] != first:
        path.append(parent[path[-1]])
    points = [tuple(start)] + [position(c) for c in reversed(path)][1:-1] + [tuple(goal)]
    # Shorten only when the entire new segment has been collision-checked.
    route, index = [points[0]], 0
    while index < len(points) - 1:
        candidate = len(points) - 1
        while not visible(points[index], points[candidate]):
            candidate -= 1
        route.append(points[candidate])
        index = candidate
    return route
