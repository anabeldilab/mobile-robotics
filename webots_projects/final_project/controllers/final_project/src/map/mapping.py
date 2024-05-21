"""
Robótica móvil
Master en informática industrial y robótica
Universidade da Coruña
Author: Anabel Díaz Labrador
        Jaime Pablo Pérez Moro

Environment mapping for the robot Khepera IV in Webots
"""

import matplotlib.pyplot as plt
import numpy as np

MAP_SIZE = 25

def create_map():
    """
    Create an empty map of the environment.

    Returns:
    - env_map: the map of the environment
    """

    # Map the environment with matrix 25x25
    # 0: free space
    # 1: wall
    # 2: start position

    env_map = [[0 for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]
    # Set start position
    env_map[int(MAP_SIZE / 2)][int(MAP_SIZE / 2)] = 2
    return env_map


def display_map(map_matrix):
    """
    Display the map of the environment.

    Parameters:
    - map_matrix: the map of the environment
    """
    map_array = np.array(map_matrix) 

    cmap = plt.cm.viridis
    norm = plt.Normalize(vmin=map_array.min(), vmax=map_array.max())

    fig, ax = plt.subplots()
    cax = ax.matshow(map_array, cmap=cmap, norm=norm)
    fig.colorbar(cax)

    ax.set_title("Robot Environment Map")
    plt.xlabel("X coordinate")
    plt.ylabel("Y coordinate")
    plt.show()


def update_map(
    env_map,
    current_position,
    current_orientation,
    front_ir,
    left_ir,
    right_ir,
    back_ir,
    front_cam_yellow_block,
):
    """
    Update the map with the position of the robot and detected walls based on IR sensor values.

    Parameters:
    - env_map: the map of the environment
    - current_position: the current position of the robot
    - current_orientation: the current orientation of the robot
    - front_ir: the front IR sensor
    - left_ir: the left IR sensor
    - right_ir: the right IR sensor

    Returns:
    - env_map_copy: the updated map of the environment
    """
    # Map sensor values to variables for clarity and efficiency
    front_value = front_ir.getValue()
    left_value = left_ir.getValue()
    right_value = right_ir.getValue()
    back_value = back_ir.getValue()

    # Define relative positions for sensors based on orientation
    sensor_positions = {
        "N": (
            (-1, 0, front_value),
            (0, -1, left_value),
            (0, 1, right_value),
            (1, 0, back_value),
        ),
        "E": (
            (0, 1, front_value),
            (-1, 0, left_value),
            (1, 0, right_value),
            (0, -1, back_value),
        ),
        "S": (
            (1, 0, front_value),
            (0, 1, left_value),
            (0, -1, right_value),
            (-1, 0, back_value),
        ),
        "W": (
            (0, -1, front_value),
            (1, 0, left_value),
            (-1, 0, right_value),
            (0, 1, back_value),
        ),
    }

    env_map_copy = env_map.copy()

    # Process each sensor's value based on current orientation
    for index, (dx, dy, value) in enumerate(sensor_positions[current_orientation]):
        nx, ny = current_position[0] + dx, current_position[1] + dy
        # env_map_copy[current_position[0]][current_position[1]] = 2
        # Check if the new position is within the map boundaries
        if 0 <= nx < len(env_map) and 0 <= ny < len(env_map[0]):
            # If it's the first iteration (front sensor) and a yellow block is detected, 
            # mark the map
            if index == 0 and front_cam_yellow_block and value >= 200:
                env_map_copy[nx][ny] = 3
            if value >= 200 and env_map_copy[nx][ny] != 3:
                env_map_copy[nx][ny] = 1

    # Set the robot's starting position (consider using a parameter for the map center)
    map_center = (len(env_map) // 2, len(env_map[0]) // 2)
    env_map_copy[map_center[0]][map_center[1]] = 2

    return env_map_copy


def fill_map(matrix):
    """Fill the map with walls using an iterative DFS algorithm.
    Parameters:
    - matrix: the map of the environment

    Returns:
    - matrix: the updated map of the environment
    """
    rows, cols = len(matrix), len(matrix[0])

    # Function to perform the DFS iteratively
    def dfs(x, y):
        stack = [(x, y)]
        while stack:
            cx, cy = stack.pop()
            if (
                cx < 0
                or cx >= rows
                or cy < 0
                or cy >= cols
                or matrix[cx][cy] == 1
                or matrix[cx][cy] == 3
            ):
                continue
            matrix[cx][cy] = 1  # Mark as visited and change to 1
            # Push adjacent cells onto the stack
            stack.append((cx + 1, cy))
            stack.append((cx - 1, cy))
            stack.append((cx, cy + 1))
            stack.append((cx, cy - 1))

    # Call DFS from the edges of the matrix
    for i in range(rows):
        if matrix[i][0] == 0:
            dfs(i, 0)
        if matrix[i][cols - 1] == 0:
            dfs(i, cols - 1)
    for j in range(cols):
        if matrix[0][j] == 0:
            dfs(0, j)
        if matrix[rows - 1][j] == 0:
            dfs(rows - 1, j)

    return matrix
