import heapq
import math

def dijkstra(env_map, start, goal):
    """
    Implementation of Dijkstra's algorithm to find the shortest path using 8 cardinal directions.

    Parameters:
    - env_map: 2D list representing the map of the environment
    - start: tuple (x, y) representing the starting position
    - goal: tuple (x, y) representing the goal position

    Returns:
    - path: list of tuples representing the path from start to goal
    """
    DIAGONAL_COST = math.sqrt(2)
    rows, cols = len(env_map), len(env_map[0])
    distances = [[float("inf")] * cols for _ in range(rows)]
    distances[start[0]][start[1]] = 0
    priority_queue = [(0, start)]
    came_from = {start: None}

    while priority_queue:
        current_distance, current_position = heapq.heappop(priority_queue)
        if current_position == goal:
            break

        neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        for dx, dy in neighbors:
            neighbor = (current_position[0] + dx, current_position[1] + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if env_map[neighbor[0]][neighbor[1]] not in (1, 3):
                    if abs(dx) == 1 and abs(dy) == 1:  # Diagonal movement
                        if env_map[current_position[0]][current_position[1] + dy] in (1, 3) or env_map[current_position[0] + dx][current_position[1]] in (1, 3):
                            continue
                        distance = current_distance + DIAGONAL_COST
                    else:
                        distance = current_distance + 1

                    if distance < distances[neighbor[0]][neighbor[1]]:
                        distances[neighbor[0]][neighbor[1]] = distance
                        heapq.heappush(priority_queue, (distance, neighbor))
                        came_from[neighbor] = current_position

    # Reconstruct path
    path = []
    step = goal
    while step is not None:
        path.append(step)
        step = came_from.get(step)
    path.reverse()

    return path if path and path[0] == start else []