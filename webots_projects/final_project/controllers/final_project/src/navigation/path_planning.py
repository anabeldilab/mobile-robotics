from src.navigation.path_planning_algorithms.dijkstra import dijkstra
import math

class PathPlanning:
    def __init__(self, robot):
        """
        Initialize the PathPlanning class.

        Parameters:
        - robot: Instance of the robot.
        - devices: Instance of RobotDevices containing the robot's devices.
        - move_forward: Instance of MoveForward class for handling forward movement.
        - turn: Instance of Turn class for handling turns.
        - env_map: 2D list representing the map of the environment
        """
        self.robot = robot

    def find_shortest_path(self, start, goal, neighbor_type='8-way'):
        """
        Find the shortest path from start to goal using Dijkstra's algorithm.

        Parameters:
        - start: tuple (x, y) representing the starting position
        - goal: tuple (x, y) representing the goal position

        Returns:
        - path: list of tuples representing the path from start to goal
        """
        print("Finding shortest path using Dijkstra's algorithm")
        return dijkstra(self.robot.mapping, start, goal, neighbor_type)

    def move_along_path(self, path, current_orientation='N'):
        """
        Move the robot along the given path.

        Parameters:
        - path: list of tuples representing the path from start to goal
        """
        print("Moving along path")
        print(f"Path: {path}")
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            print(f"dx: {dx}, dy: {dy}")
            target_orientation = self.determine_orientation(dx, dy)
            print(f"current_orientation: {current_orientation}, target_orientation: {target_orientation}")

            self.adjust_orientation(current_orientation, target_orientation)
            current_orientation = target_orientation

            if abs(dx) + abs(dy) == 2:  # Diagonal movement
                self.robot.move_forward.execute(0.25 * math.sqrt(2))
            else:
                self.robot.move_forward.execute()

    def determine_orientation(self, dx, dy):
        if dx == 0 and dy == 1:
            return 'E'
        elif dx == 1 and dy == 1:
            return 'SE'
        elif dx == 1 and dy == 0:
            return 'S'
        elif dx == 1 and dy == -1:
            return 'SW'
        elif dx == 0 and dy == -1:
            return 'W'
        elif dx == -1 and dy == -1:
            return 'NW'
        elif dx == -1 and dy == 0:
            return 'N'
        elif dx == -1 and dy == 1:
            return 'NE'
        else:
            return None

    def adjust_orientation(self, current_orientation, target_orientation):
        """
        Adjust the robot's orientation to the target orientation.

        Parameters:
        - current_orientation: Current orientation of the robot
        - target_orientation: Target orientation of the robot
        """
        orientation_map = {
            ('N', 'NE'): ('right', 45), ('N', 'E'): ('right', 90), ('N', 'SE'): ('right', 135),
            ('N', 'S'): ('right', 180), ('N', 'SW'): ('left', 135), ('N', 'W'): ('left', 90),
            ('N', 'NW'): ('left', 45),

            ('NE', 'E'): ('right', 45), ('NE', 'SE'): ('right', 90), ('NE', 'S'): ('right', 135),
            ('NE', 'SW'): ('right', 180), ('NE', 'W'): ('left', 135), ('NE', 'NW'): ('left', 90),
            ('NE', 'N'): ('left', 45),

            ('E', 'SE'): ('right', 45), ('E', 'S'): ('right', 90), ('E', 'SW'): ('right', 135),
            ('E', 'W'): ('right', 180), ('E', 'NW'): ('left', 135), ('E', 'N'): ('left', 90),
            ('E', 'NE'): ('left', 45),

            ('SE', 'S'): ('right', 45), ('SE', 'SW'): ('right', 90), ('SE', 'W'): ('right', 135),
            ('SE', 'NW'): ('right', 180), ('SE', 'N'): ('left', 135), ('SE', 'NE'): ('left', 90),
            ('SE', 'E'): ('left', 45),

            ('S', 'SW'): ('right', 45), ('S', 'W'): ('right', 90), ('S', 'NW'): ('right', 135),
            ('S', 'N'): ('right', 180), ('S', 'NE'): ('left', 135), ('S', 'E'): ('left', 90),
            ('S', 'SE'): ('left', 45),

            ('SW', 'W'): ('right', 45), ('SW', 'NW'): ('right', 90), ('SW', 'N'): ('right', 135),
            ('SW', 'NE'): ('right', 180), ('SW', 'E'): ('left', 135), ('SW', 'SE'): ('left', 90),
            ('SW', 'S'): ('left', 45),

            ('W', 'NW'): ('right', 45), ('W', 'N'): ('right', 90), ('W', 'NE'): ('right', 135),
            ('W', 'E'): ('right', 180), ('W', 'SE'): ('left', 135), ('W', 'S'): ('left', 90),
            ('W', 'SW'): ('left', 45),

            ('NW', 'N'): ('right', 45), ('NW', 'NE'): ('right', 90), ('NW', 'E'): ('right', 135),
            ('NW', 'SE'): ('right', 180), ('NW', 'S'): ('left', 135), ('NW', 'SW'): ('left', 90),
            ('NW', 'W'): ('left', 45)
        }
        print(f"Adjusting orientation from {current_orientation} to {target_orientation}")
        if current_orientation == target_orientation:
            return

        if (current_orientation, target_orientation) in orientation_map:
            direction, angle = orientation_map[(current_orientation, target_orientation)]
            self.robot.turn.execute(direction, angle)
        else:
            print(f"Invalid orientation transition from {current_orientation} to {target_orientation}")

    def execute(self, start, goal, current_orientation, neighbor_type='8-way'):
        """
        Execute the path planning and move the robot to the goal.

        Parameters:
        - start: tuple (x, y) representing the starting position
        - goal: tuple (x, y) representing the goal position
        """
        path = self.find_shortest_path(start, goal, neighbor_type)
        print(f"Start: {start}, Goal: {goal}, Path: {path}")
        self.robot.mapping.update_path(path)
        self.robot.mapping.display_map()

        self.move_along_path(path, current_orientation)

