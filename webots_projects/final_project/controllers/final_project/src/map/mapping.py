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
import pickle
from collections import deque


class Mapping:
    FREE_SPACE = 0
    WALL = 1
    START_POSITION = 2
    YELLOW_BLOCK = 3
    PATH = 4

    def __init__(self, filename=None, size=[25, 25]):
        if filename:
            self.data = self.load_map(filename)
        else:
            self.size = size
            self.data = self.create_map()
        self.previous_values = {}
        self.yellow_block_position = None

    def is_transitable(self, x, y):
        """
        Check if a given position is transitable.

        Parameters:
        - x: The x coordinate on the map.
        - y: The y coordinate on the map.

        Returns:
        - bool: True if the position is transitable, False otherwise.
        """
        return (
            0 <= x < len(self.data)
            and 0 <= y < len(self.data[0])
            and self.data[x][y] != self.WALL
            and self.data[x][y] != self.YELLOW_BLOCK
        )

    def create_map(self):
        """
        Create an empty map of the environment.

        Returns:
        - data: the map of the environment
        """
        data = [
            [0 for _ in range(self.size[0])] for _ in range(self.size[1])
        ]
        data[int(self.size[0] / 2)][
            int(self.size[1] / 2)
        ] = self.START_POSITION
        return data

    def update_position(self, x, y, value):
        """
        Update the map at a specific position with a given value.

        Parameters:
        - x: The x coordinate on the map.
        - y: The y coordinate on the map.
        - value: The value to set at the specified position.
        """
        if 0 <= x < len(self.data) and 0 <= y < len(self.data[0]):
            if x == int(self.size[0] / 2) and y == int(self.size[1] / 2):
                self.data[x][y] = self.START_POSITION
            else:
                self.data[x][y] = value

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
        self.previous_values[(x, y)] = self.data[x][y]
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

    def remove_previous_path(self):
        """
        Remove the previous path from the map.
        """
        for i, row in enumerate(self.data):
            for j, value in enumerate(row):
                if value == self.PATH and self.data[i][j] != self.START_POSITION:
                    self.data[i][j] = self.FREE_SPACE

    def update_path(self, path):
        """
        Update the map with the shortest path from start to goal.

        Parameters:
        - path: list of tuples representing the path from start to goal
        """
        self.remove_previous_path()
        for position in path:
            if self.data[position[0]][position[1]] != self.START_POSITION:
                self.data[position[0]][position[1]] = self.PATH

    def save_map(self, filename="src/map/maps/map.pkl"):
        """
        Save the current map data to a file using serialization.

        This method uses the pickle library to serialize the map data structure,
        which allows the complex data types within the map to be saved accurately.
        It's especially useful for preserving the state of a simulation or an ongoing
        robotic task.

        Parameters:
        - filename: str, optional
            The name of the file where the map data will be saved. The default filename is 'map.pkl'.

        Returns:
        None
        """
        with open(filename, "wb") as f:
            pickle.dump(self.data, f)
        print(f"Map saved to {filename}")

    def load_map(self, filename="src/map/maps/map.pkl"):
        """
        Load map data from a file using serialization.

        Attempts to open the specified file and deserialize the map data structure using pickle.
        If the file does not exist, it calls the create_map method to generate a new map,
        which is useful for starting with a default map configuration in case of missing files.

        Parameters:
        - filename: str, optional
            The name of the file to load the map data from. The default filename is 'map.pkl'.

        Returns:
        - data: list of lists
            The map data structure loaded from the file or newly created if the file was not found.
        """
        try:
            with open(filename, "rb") as f:
                self.data = pickle.load(f)
            print(f"Map loaded from {filename}")
            # Look for the yellow block and set the closest valid block
            self.yellow_block_position = None
            for i, row in enumerate(self.data):
                for j, value in enumerate(row):
                    if value == self.YELLOW_BLOCK:
                        self.yellow_block_position = [i, j]
                        break
            return self.data
        except FileNotFoundError:
            print("File not found. Creating a new map.")
            return self.create_map()

    def fill_map(self):
        """Fill the map with walls using an iterative DFS algorithm.
        Parameters:
        - matrix: the map of the environment

        Returns:
        - matrix: the updated map of the environment
        """
        rows, cols = len(self.data), len(self.data[0])

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
                    or self.data[cx][cy] == self.WALL
                    or self.data[cx][cy] == self.YELLOW_BLOCK
                ):
                    continue
                self.data[cx][cy] = self.WALL
                # Push adjacent cells onto the stack
                stack.append((cx + 1, cy))
                stack.append((cx - 1, cy))
                stack.append((cx, cy + 1))
                stack.append((cx, cy - 1))

        # Call DFS from the edges of the matrix
        for i in range(rows):
            if self.data[i][0] == 0:
                dfs(i, 0)
            if self.data[i][cols - 1] == 0:
                dfs(i, cols - 1)
        for j in range(cols):
            if self.data[0][j] == 0:
                dfs(0, j)
            if self.data[rows - 1][j] == 0:
                dfs(rows - 1, j)

        return self.data

    def display_map(self):
        map_array = np.array(self.data)

        cmap = plt.cm.viridis
        norm = plt.Normalize(vmin=map_array.min(), vmax=map_array.max())

        fig, ax = plt.subplots()
        cax = ax.matshow(map_array, cmap=cmap, norm=norm)
        fig.colorbar(cax)

        ax.set_title("Robot Environment Map")
        plt.xlabel("X coordinate")
        plt.ylabel("Y coordinate")

        yellow_block_present = any(self.YELLOW_BLOCK in row for row in self.data)
        if not yellow_block_present:
            print("No yellow block found, enabling manual addition.")
            self.add_yellow_block_manually(fig, ax, cax)
            plt.show()
            return self.yellow_block_position
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
                cax.set_data(np.array(self.data))
                plt.draw()

        fig.canvas.mpl_connect("button_press_event", onclick)
        ax.text(
            0.5,
            -0.1,
            "No goal found\nLeft click to set a YELLOW_BLOCK",
            ha="center",
            va="center",
            transform=ax.transAxes,
            color="red",
        )

    def has_free_space(self):
        """Check if there is free space in the map."""
        return any(self.FREE_SPACE in row for row in self.data)
