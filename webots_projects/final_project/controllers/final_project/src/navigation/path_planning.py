from src.navigation.path_planning_algorithms.dijkstra import dijkstra

class PathPlanning:
    def __init__(self, robot, devices, move_forward, turn, env_map):
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
        self.devices = devices
        self.move_forward = move_forward
        self.turn = turn
        self.env_map = env_map

    def find_shortest_path(self, start, goal):
        """
        Find the shortest path from start to goal using Dijkstra's algorithm.

        Parameters:
        - start: tuple (x, y) representing the starting position
        - goal: tuple (x, y) representing the goal position

        Returns:
        - path: list of tuples representing the path from start to goal
        """
        return dijkstra(self.env_map, start, goal)

    def move_along_path(self, path, current_orientation='N'):
        """
        Move the robot along the given path.

        Parameters:
        - path: list of tuples representing the path from start to goal
        """
        print("Moving along path")
        print(f"current_orientation: {current_orientation}")
        for i in range(1, len(path)):
            next_position = path[i]
            if next_position[0] < path[i-1][0]:
                target_orientation = 'N'
            elif next_position[0] > path[i-1][0]:
                target_orientation = 'S'
            elif next_position[1] < path[i-1][1]:
                target_orientation = 'W'
            else:
                target_orientation = 'E'
            print(f"target_orientation: {target_orientation}")

            self.adjust_orientation(current_orientation, target_orientation)
            current_orientation = target_orientation
            self.move_forward.execute()

    def adjust_orientation(self, current_orientation, target_orientation):
        """
        Adjust the robot's orientation to the target orientation.

        Parameters:
        - current_orientation: Current orientation of the robot
        - target_orientation: Target orientation of the robot
        """
        if current_orientation == target_orientation:
            return
        elif (current_orientation, target_orientation) in [('N', 'E'), ('E', 'S'), ('S', 'W'), ('W', 'N')]:
            self.turn.execute('right')
            print("right")
        elif (current_orientation, target_orientation) in [('N', 'W'), ('W', 'S'), ('S', 'E'), ('E', 'N')]:
            self.turn.execute('left')
            print("left")
        else:
            self.turn.execute('right')
            self.turn.execute('right')
            print("right right")

    def execute(self, start, goal, current_orientation):
        """
        Execute the path planning and move the robot to the goal.

        Parameters:
        - start: tuple (x, y) representing the starting position
        - goal: tuple (x, y) representing the goal position
        """
        path = self.find_shortest_path(start, goal)
        self.move_along_path(path, current_orientation)

