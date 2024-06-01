from controller import Supervisor
from src.robot.robot_devices import RobotDevices

from src.motion.supervisor.turn import Turn
from src.motion.supervisor.move import MoveForward
from src.navigation.wall_follow import WallFollower
from src.map.mapping import Mapping
from src.motion.supervisor.orientation import Orientation
from src.navigation.path_planning import PathPlanning

class KheperaSupervisor:
    def __init__(self, time_step=32, max_speed=47.6, map_width=25, map_height=25):
        """
        Initialize the KheperaSupervisor class.

        Parameters:
        - time_step: Time step for sensor and actuator updates.
        - max_speed: Maximum speed of the robot.
        """
        self.robot = Supervisor()
        self.robot_node = self.robot.getFromDef("Khepera")
        self.devices = RobotDevices(time_step, self.robot)
        self.move_forward = MoveForward(self)
        self.turn = Turn(self)
        self.mapping = Mapping(size=[map_width, map_height])
        self.wall_follower = WallFollower(self)
        self.path_planning = PathPlanning(self)

        self.start_position = [int(map_height / 2), int(map_width / 2)]
        self.current_position = self.start_position.copy()
        self.current_orientation = Orientation.get_orientation(self.robot_node)
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

        for direction, (dx, dy) in sensor_positions[self.current_orientation].items():
            if direction in walls:
                nx, ny = self.current_position[0] + dx, self.current_position[1] + dy
                if  self.yellow_block != [nx, ny]:
                    self.mapping.update_wall(nx, ny)

        if yellow_block_detected:
            front_dx, front_dy = sensor_positions[self.current_orientation]['front']
            nx, ny = self.current_position[0] + front_dx, self.current_position[1] + front_dy
            self.yellow_block = [nx, ny]
            self.mapping.update_yellow_block(nx, ny)

    def change_position(self):
        """
        Change the position of the robot based on the current orientation in a grid.

        Parameters:
        - current_position (list of int): The current position of the robot as a list [row, column].
        - orientation (str): The current orientation of the robot, expected to be 'N', 'E', 'S', or 'W'.

        Returns:
        - list of int: Updated position of the robot after moving according to the orientation.

        Raises:
        - ValueError: If an invalid orientation is provided.
        """
        if not isinstance(self.current_position, list) or len(self.current_position) != 2:
            raise ValueError("current_position must be a list of two integers")
        if self.current_orientation not in ("N", "E", "S", "W"):
            raise ValueError("Invalid orientation. Must be 'N', 'E', 'S', or 'W'.")

        if self.current_orientation == "N":
            self.current_position[0] -= 1  # Move up in the grid (decrease row index)
        elif self.current_orientation == "E":
            self.current_position[1] += 1  # Move right in the grid (increase column index)
        elif self.current_orientation == "S":
            self.current_position[0] += 1  # Move down in the grid (increase row index)
        elif self.current_orientation == "W":
            self.current_position[1] -= 1  # Move left in the grid (decrease column index)

        return self.current_position

    def run(self):
        """Run the wall following behavior."""
        self.wall_follower.follow_wall()

        self.mapping.fill_map()

        if not self.mapping.has_free_space():
            print("Incomplete map, loading map data...")
            self.mapping.load_map("src/map/maps/map_data.txt")
        else:
            self.mapping.save_map("src/map/maps/map_data.txt")

        start = (self.start_position[0], self.start_position[1])
        goal = self.mapping.display_map()
        if goal is not None:
            self.yellow_block = goal

        goal = (self.yellow_block[0], self.yellow_block[1])

        self.path_planning.execute(start, goal, self.current_orientation, neighbor_type="4-way")

        start = (self.current_position[0], self.current_position[1])
        print(f"Start position must be current position: {start}")
        goal = (self.start_position[0], self.start_position[1])

        self.path_planning.execute(start, goal, self.current_orientation, neighbor_type="4-way")
