from controller import Robot
from src.robot.robot_devices import RobotDevices

from src.motion.odometry.turn import Turn
from src.motion.odometry.move import MoveForward
from src.map.mapping import Mapping
from src.navigation.wall_follow import WallFollower
from src.navigation.path_planning import PathPlanning

class Khepera:
    def __init__(self, time_step=32, max_speed=10, map_width=25, map_height=25):
        """
        Initialize the Khepera class.

        Parameters:
        - time_step: Time step for sensor and actuator updates.
        - max_speed: Maximum speed of the robot.
        """
        self.robot = Robot()
        self.devices = RobotDevices(time_step, self.robot)
        self.move_forward = MoveForward(self)
        self.turn = Turn(self)
        self.mapping = Mapping(map_size=[map_width, map_height])
        self.wall_follower = WallFollower(self)
        self.path_planning = PathPlanning(self)

        self.start_position = [int(map_height / 2), int(map_width / 2)]
        self.current_position = self.start_position.copy()
        self.current_cardinal = "N"
        self.goal_position = None
        self.yellow_block = None

    def step(self):
        """Step the simulation."""
        return self.robot.step(self.devices.time_step)

    def collect_sensor_data(self):
        """
        Collect data from sensors and return in a structured format.
        """
        front_ir = self.devices.ir_sensor_list["front infrared sensor"].getValue()
        left_ir = self.devices.ir_sensor_list["left infrared sensor"].getValue()
        right_ir = self.devices.ir_sensor_list["right infrared sensor"].getValue()
        back_ir = self.devices.ir_sensor_list["rear infrared sensor"].getValue()
        front_is_yellow = self.devices.detect_yellow_block()

        sensor_readings = {
            'front': front_ir,
            'left': left_ir,
            'right': right_ir,
            'back': back_ir
        }

        print(f"Sensor readings: {sensor_readings}")
        print(f"Yellow block detected: {front_is_yellow}")

        return sensor_readings, front_is_yellow

    def detect_walls(self, sensor_values):
        """
        Determine the presence of walls based on sensor values.
        Returns a dictionary with wall positions.
        """
        walls = {}
        threshold = 200
        if sensor_values['front'] >= threshold:
            walls['front'] = True
        if sensor_values['left'] >= threshold:
            walls['left'] = True
        if sensor_values['right'] >= threshold:
            walls['right'] = True
        if sensor_values['back'] >= threshold:
            walls['back'] = True
        return walls

    def update_map(self):
        """
        Collect sensor data, determine walls, and update the map.
        """
        sensor_readings, yellow_block_detected = self.collect_sensor_data()
        walls = self.detect_walls(sensor_readings)
        
        # Define relative positions for sensors based on orientation
        sensor_positions = {
            "N": {'front': (-1, 0), 'left': (0, -1), 'right': (0, 1), 'back': (1, 0)},
            "E": {'front': (0, 1), 'left': (-1, 0), 'right': (1, 0), 'back': (0, -1)},
            "S": {'front': (1, 0), 'left': (0, 1), 'right': (0, -1), 'back': (-1, 0)},
            "W": {'front': (0, -1), 'left': (1, 0), 'right': (-1, 0), 'back': (0, 1)},
        }

        for direction, (dx, dy) in sensor_positions[self.current_cardinal].items():
            if direction in walls:
                nx, ny = self.current_position[0] + dx, self.current_position[1] + dy
                if  self.yellow_block != [nx, ny]:
                    self.mapping.update_wall(nx, ny)

        if yellow_block_detected:
            front_dx, front_dy = sensor_positions[self.current_cardinal]['front']
            nx, ny = self.current_position[0] + front_dx, self.current_position[1] + front_dy
            self.yellow_block = [nx, ny]
            self.goal_position = self.current_position.copy()
            self.mapping.update_yellow_block(nx, ny)

    def change_position(self):
        """
        Change the position of the robot based on the current orientation in a grid.

        Returns:
        - list of int: Updated position of the robot after moving according to the orientation.
        """
        if (
            not isinstance(self.current_position, list)
            or len(self.current_position) != 2
        ):
            raise ValueError("current_position must be a list of two integers")
        if self.current_cardinal not in ("N", "E", "S", "W"):
            raise ValueError("Invalid current_cardinal. Must be 'N', 'E', 'S', or 'W'.")

        if self.current_cardinal == "N":
            self.current_position[0] -= 1
        elif self.current_cardinal == "E":
            self.current_position[1] += 1
        elif self.current_cardinal == "S":
            self.current_position[0] += 1
        elif self.current_cardinal == "W":
            self.current_position[1] -= 1

        return self.current_position

    def run(self):
        """Run the wall following behavior."""
        self.wall_follower.follow_wall()

        self.mapping.fill_map()
        start = (self.start_position[0], self.start_position[1])
        goal = self.mapping.display_map()
        print(f"Goal position: {self.goal_position}")
        print(f"Display map returned goal: {goal}")

        if goal is not None:
            self.goal_position = goal
        print(f"Updated goal position: {self.goal_position}")

        goal = (self.goal_position[0], self.goal_position[1])
        
        print(f"Hola Start: {self.start_position}, Goal: {self.goal_position}")
        print(f"Map: {self.mapping.map_data}")

        path = self.path_planning.find_shortest_path(start, goal)
        if path is None:
            print(f"Path not found from {start} to {goal}")
        else:
            print(f"Path found: {path}")

        if path is not None:
            self.path_planning.execute(start, goal, self.current_cardinal)
        else:
            print("No valid path found!")

