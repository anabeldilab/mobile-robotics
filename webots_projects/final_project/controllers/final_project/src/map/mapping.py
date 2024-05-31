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
from collections import deque

class Mapping:
    FREE_SPACE = 0
    WALL = 1
    START_POSITION = 2
    YELLOW_BLOCK = 3
    PATH = 4

    def __init__(self, filename=None, map_size=[25, 25]):
        if filename:
            self.map_data = self.load_map(filename)
        else:
            self.map_size = map_size
            self.map_data = self.create_map()
        self.previous_values = {}
        self.yellow_block_position = None
        self.closest_valid_block = None

    def create_map(self):
        """
        Create an empty map of the environment.

        Returns:
        - map_data: the map of the environment
        """
        map_data = [[0 for _ in range(self.map_size[0])] for _ in range(self.map_size[1])]
        map_data[int(self.map_size[0] / 2)][int(self.map_size[1] / 2)] = self.START_POSITION
        return map_data

    def update_position(self, x, y, value):
        """
        Update the map at a specific position with a given value.

        Parameters:
        - x: The x coordinate on the map.
        - y: The y coordinate on the map.
        - value: The value to set at the specified position.
        """
        if 0 <= x < len(self.map_data) and 0 <= y < len(self.map_data[0]):
            if x == int(self.map_size[0] / 2) and y == int(self.map_size[1] / 2):
                self.map_data[x][y] = self.START_POSITION
            else:
                self.map_data[x][y] = value

    def update_wall(self, x, y):
        """
        Update the map with a wall at a specific position.

        Parameters:
        - x: The x coordinate on the map.
        - y: The y coordinate on the map.
        """
        self.update_position(x, y, self.WALL)

    def update_yellow_block(self, x, y):
        """
        Update the map with a yellow block at a specific position.

        Parameters:
        - x: The x coordinate on the map.
        - y: The y coordinate on the map.
        """
        self.previous_values[(x, y)] = self.map_data[x][y]
        self.update_position(x, y, self.YELLOW_BLOCK)

    def remove_yellow_block(self, x, y):
        """
        Remove the yellow block from the map, restoring the previous value.

        Parameters:
        - x: The x coordinate on the map.
        - y: The y coordinate on the map.
        """
        if (x, y) in self.previous_values:
            self.update_position(x, y, self.previous_values.pop((x, y)))

    def update_path(self, path):
        """
        Update the map with the shortest path from start to goal.

        Parameters:
        - path: list of tuples representing the path from start to goal
        """
        for position in path:
            self.map_data[position[0]][position[1]] = self.PATH

    def save_map(self):
        # Lógica para guardar el mapa en un archivo
        pass

    def load_map(self, filename):
        # Lógica para cargar el mapa desde un archivo
        pass

    def fill_map(self):
        """Fill the map with walls using an iterative DFS algorithm.
        Parameters:
        - matrix: the map of the environment

        Returns:
        - matrix: the updated map of the environment
        """
        rows, cols = len(self.map_data), len(self.map_data[0])

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
                    or self.map_data[cx][cy] == self.WALL
                    or self.map_data[cx][cy] == self.YELLOW_BLOCK
                ):
                    continue
                self.map_data[cx][cy] = self.WALL
                # Push adjacent cells onto the stack
                stack.append((cx + 1, cy))
                stack.append((cx - 1, cy))
                stack.append((cx, cy + 1))
                stack.append((cx, cy - 1))

        # Call DFS from the edges of the matrix
        for i in range(rows):
            if self.map_data[i][0] == 0:
                dfs(i, 0)
            if self.map_data[i][cols - 1] == 0:
                dfs(i, cols - 1)
        for j in range(cols):
            if self.map_data[0][j] == 0:
                dfs(0, j)
            if self.map_data[rows - 1][j] == 0:
                dfs(rows - 1, j)

        return self.map_data

    def display_map(self):
        map_array = np.array(self.map_data)

        cmap = plt.cm.viridis
        norm = plt.Normalize(vmin=map_array.min(), vmax=map_array.max())

        fig, ax = plt.subplots()
        cax = ax.matshow(map_array, cmap=cmap, norm=norm)
        fig.colorbar(cax)

        ax.set_title("Robot Environment Map")
        plt.xlabel("X coordinate")
        plt.ylabel("Y coordinate")

        yellow_block_present = any(self.YELLOW_BLOCK in row for row in self.map_data)
        if not yellow_block_present:
            print("No yellow block found, enabling manual addition.")
            self.add_yellow_block_manually(fig, ax, cax)
            plt.show()
            return self.closest_valid_block
        else:
            plt.show()
            return None

    def add_yellow_block_manually(self, fig, ax, cax):
        print("Setting up event handler for manual yellow block addition.")

        def onclick(event):
            if event.xdata is not None and event.ydata is not None:
                x = int(event.ydata)
                y = int(event.xdata)
                print(f"Click event: button={event.button}, x={x}, y={y}")
                if event.button == 1:
                    if self.yellow_block_position:
                        last_x, last_y = self.yellow_block_position
                        self.remove_yellow_block(x=last_x, y=last_y)
                    self.update_yellow_block(x=x, y=y)
                    self.yellow_block_position = [x, y]
                    self.closest_valid_block = self.find_closest_valid_block()
                    print("Closest valid block:", self.closest_valid_block)
                cax.set_data(np.array(self.map_data))
                plt.draw()

        fig.canvas.mpl_connect('button_press_event', onclick)
        print("On click return:", self.closest_valid_block)
        ax.text(0.5, -0.1, "No goal found\nLeft click to set a YELLOW_BLOCK", ha='center', va='center', transform=ax.transAxes, color='red')

    def find_closest_valid_block(self):
        """Find the closest valid block to the yellow block."""
        if not self.yellow_block_position:
            return None

        rows, cols = len(self.map_data), len(self.map_data[0])
        visited = [[False for _ in range(cols)] for _ in range(rows)]
        queue = deque([self.yellow_block_position])
        while queue:
            x, y = queue.popleft()
            if self.map_data[x][y] == self.FREE_SPACE:
                return [x, y]
            visited[x][y] = True
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < rows and 0 <= ny < cols and not visited[nx][ny]:
                    queue.append((nx, ny))
        return None