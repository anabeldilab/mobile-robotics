import heapq
import math

def dijkstra(mapping, start, goal, neighbor_type='8-way'):
    """
    Implementation of Dijkstra's algorithm to find the shortest path.
    Allows for both orthogonal (4-way) and diagonal (8-way) neighbor considerations.

    Parameters:
    - mapping: Map class instance representing the environment
    - start: tuple (x, y) representing the starting position
    - goal: tuple (x, y) representing the position of the yellow block (goal)
    - neighbor_type: '4-way' for orthogonal neighbors, '8-way' for diagonal neighbors

    Returns:
    - path: list of tuples representing the path from start to the goal or nearest reachable position
    """
    DIAGONAL_COST = math.sqrt(2)
    rows, cols = mapping.size[0], mapping.size[1]
    distances = [[float("inf")] * cols for _ in range(rows)]
    distances[start[0]][start[1]] = 0
    priority_queue = [(0, start)]
    came_from = {start: None}
    goal_traversable = mapping.is_transitable(goal[0], goal[1])

    if neighbor_type == '4-way':
        neighbors = [(0, -1), (0, 1), (-1, 0), (1, 0)]
    else:
        neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    while priority_queue:
        current_distance, current_position = heapq.heappop(priority_queue)
        
        if goal_traversable and current_position == goal:
            break
        
        if not goal_traversable and (abs(current_position[0] - goal[0]) + abs(current_position[1] - goal[1]) == 1):
            if mapping.data[current_position[0]][current_position[1]] == 0:
                break

        for dx, dy in neighbors:
            neighbor = (current_position[0] + dx, current_position[1] + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and mapping.data[neighbor[0]][neighbor[1]] != 1:
                if abs(dx) == 1 and abs(dy) == 1: # Diagonal movement
                    if mapping.data[current_position[0]][neighbor[1]] == 1 or mapping.data[neighbor[0]][current_position[1]] == 1:
                        continue
                    distance = current_distance + DIAGONAL_COST
                else:
                    distance = current_distance + 1

                if distance < distances[neighbor[0]][neighbor[1]]:
                    distances[neighbor[0]][neighbor[1]] = distance
                    heapq.heappush(priority_queue, (distance, neighbor))
                    came_from[neighbor] = current_position

    # Reconstruct path from goal to start using the came_from map
    path = []
    step = current_position
    while step is not None:
        path.append(step)
        step = came_from.get(step)
    path.reverse()

    return path if path and path[0] == start else []

