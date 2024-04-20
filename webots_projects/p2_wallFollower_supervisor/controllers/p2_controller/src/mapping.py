"""
Robótica móvil
Master en informática industrial y robótica
Universidade da Coruña
Author: Anabel Díaz Labrador
        Jaime Pablo Pérez Moro

Creación de un mapa del entorno.
"""

import matplotlib.pyplot as plt
import numpy as np

ORIENTATIONS = {
    "N": [[0, -1], [-1, 0]],
    "E": [[1, 0], [0, -1]],
    "S": [[0, 1], [1, 0]],
    "W": [[-1, 0], [0, 1]],
}


def create_map():
    """
    Create an empty map of the environment.
    """

    # Map the environment with matrix 25x25
    # 0: free space
    # 1: wall
    # 2: start position

    env_map = [[0 for _ in range(25)] for _ in range(25)]
    # Set start position
    env_map[int(25 / 2)][int(25 / 2)] = 2
    return env_map


def display_map(map_matrix):
    """
    Display the map of the environment.
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
    env_map, current_position, current_orientation, front_ir, left_ir, right_ir
):
    """
    Update the map with the position of the robot.
    """
    # Update the map with the position of the robot
    # 0: free space
    # 1: wall
    # 2: start position
    # Update the map with the position of the walls using current position and orientation
    env_map_copy = env_map.copy()
    if current_orientation == "N":
        print("N")
        if front_ir.getValue() >= 180:
            env_map_copy[current_position[0] - 1][current_position[1]] = 1
        if left_ir.getValue() >= 180:
            env_map_copy[current_position[0]][current_position[1] - 1] = 1
        if right_ir.getValue() >= 180:
            env_map_copy[current_position[0]][current_position[1] + 1] = 1

    elif current_orientation == "E":
        print("E")
        if front_ir.getValue() >= 180:
            env_map_copy[current_position[0]][current_position[1] + 1] = 1
        if left_ir.getValue() >= 180:
            env_map_copy[current_position[0] - 1][current_position[1]] = 1
        if right_ir.getValue() >= 180:
            env_map_copy[current_position[0] + 1][current_position[1]] = 1

    elif current_orientation == "S":
        print("S")
        if front_ir.getValue() >= 180:
            env_map_copy[current_position[0] + 1][current_position[1]] = 1
        if left_ir.getValue() >= 180:
            env_map_copy[current_position[0]][current_position[1] + 1] = 1
        if right_ir.getValue() >= 180:
            env_map_copy[current_position[0]][current_position[1] - 1] = 1

    elif current_orientation == "W":
        print("W")
        if front_ir.getValue() >= 180:
            env_map_copy[current_position[0]][current_position[1] - 1] = 1
        if left_ir.getValue() >= 180:
            env_map_copy[current_position[0] + 1][current_position[1]] = 1
        if right_ir.getValue() >= 180:
            env_map_copy[current_position[0] - 1][current_position[1]] = 1
    
    env_map_copy[int(25 / 2)][int(25 / 2)] = 2

    return env_map_copy


def fill_map(matrix):
    rows, cols = len(matrix), len(matrix[0])
    
    # Función para realizar la DFS
    def dfs(x, y):
        if x < 0 or x >= rows or y < 0 or y >= cols or matrix[x][y] == 1:
            return
        matrix[x][y] = 1  # Marcar como visitado y cambiar a 1
        # Llamadas recursivas solo en las direcciones cardinales
        dfs(x + 1, y)
        dfs(x - 1, y)
        dfs(x, y + 1)
        dfs(x, y - 1)

    # Llamar DFS para los bordes de la matriz
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
