from controller import Robot
import math

from src.map.mapping import create_map, update_map, fill_map, display_map, MAP_SIZE

# Constants
WHEEL_BASE = 0.10829  # meters (105.4 mm)
WHEEL_RADIUS = 0.021  # meters (21 mm)
TIME_STEP = 32  # milliseconds

class Khepera:
    """Khepera robot class that uses odometry and gyroscope to know its position"""

    def __init__(self, time_step=32, max_speed=10, distance_to_move=0.25):
        self.time_step = time_step
        (
            self.robot,
            self.left_wheel,
            self.right_wheel,
            self.ir_sensor_list,
            self.pos_l,
            self.pos_r,
            self.camera,
            self.gyro,
        ) = self.init_devices()

        self.max_speed = max_speed
        self.wheel_radius = WHEEL_RADIUS  # meters (21 mm)
        self.wheel_base = WHEEL_BASE  # meters (105.4 mm)
        self.distance_to_move = distance_to_move

        # ORIENTATION ATTRIBUTES
        self.current_orientation = 0.0  # radians
        self.current_cardinal = "N"
        self.current_position = [0, 0]  # x, y coordinates

    ORIENTATIONS = {
        "N": 0,
        "W": 90,
        "S": 180,
        "E": 270,
    }

    def init_devices(self):
        """
        Set up and retrieve necessary devices.

        Parameters:
        - time_step: default update interval in milliseconds for sensors/actuators.

        Returns:
        - tuple: Contains instances of the robot, wheel motors, infrared sensors, position sensors,
        and camera.
        """
        self.robot = Robot()

        left_wheel = self.robot.getDevice("left wheel motor")
        right_wheel = self.robot.getDevice("right wheel motor")
        left_wheel.setPosition(float("inf"))
        right_wheel.setPosition(float("inf"))
        left_wheel.setVelocity(0)
        right_wheel.setVelocity(0)

        ir_sensor_list = self.enable_distance_sensors()

        camera = self.robot.getDevice("camera")
        camera.enable(self.time_step * 10)

        pos_l = self.robot.getDevice("left wheel sensor")
        pos_r = self.robot.getDevice("right wheel sensor")
        pos_l.enable(self.time_step)
        pos_r.enable(self.time_step)

        self.robot.step(self.time_step)

        gyro = self.robot.getDevice("gyro")
        gyro.enable(self.time_step)

        return (
            self.robot,
            left_wheel,
            right_wheel,
            ir_sensor_list,
            pos_l,
            pos_r,
            camera,
            gyro,
        )

    def enable_distance_sensors(self):
        """
        Activate and retrieve distance sensors.

        Parameters:
        - robot: instance of the robot.
        - time_step: update interval for sensors/actuators in milliseconds.
        - sensor_names: list of sensor names to activate.

        Returns:
        - dict: Dictionary with activated distance sensors, ordered as in the sensor_names list.
        """
        sensor_names = [
            "rear left infrared sensor",
            "left infrared sensor",
            "front left infrared sensor",
            "front infrared sensor",
            "front right infrared sensor",
            "right infrared sensor",
            "rear right infrared sensor",
            "rear infrared sensor",
        ]
        sensor_dict = {}

        for name in sensor_names:
            sensor_dict[name] = self.robot.getDevice(name)
            sensor_dict[name].enable(self.time_step)

        return sensor_dict

    def move_forward(self, distance = 0.25, speed = 10):
        """
        Move the robot forward a specific distance using odometry.

        Parameters:
        - robot: instance of the robot.
        - left_wheel: left wheel motor device.
        - right_wheel: right wheel motor device.
        - pos_l: left wheel position sensor.
        - pos_r: right wheel position sensor.
        - distance: distance to move forward in meters.
        - speed: wheel speed.
        """
        # Calculate the required wheel rotation to cover the distance
        delta_position = distance / WHEEL_RADIUS

        # Get the initial wheel positions
        initial_pos_l = self.pos_l.getValue()
        initial_pos_r = self.pos_r.getValue()

        if initial_pos_l is not float or initial_pos_r is not float:
            initial_pos_l = self.pos_l.getValue()
            initial_pos_r = self.pos_r.getValue()
        
        # Set wheel speeds
        self.left_wheel.setVelocity(speed)
        self.right_wheel.setVelocity(speed)

        while self.robot.step(TIME_STEP) != -1:
            # Calculate the distance covered by each wheel
            current_pos_l = self.pos_l.getValue()
            current_pos_r = self.pos_r.getValue()
            left_distance = (current_pos_l - initial_pos_l) * WHEEL_RADIUS
            right_distance = (current_pos_r - initial_pos_r) * WHEEL_RADIUS

            # Check if the average distance covered by the wheels is greater than or equal to the target distance
            if (left_distance + right_distance) / 2.0 >= distance:
                break

        # Stop the motors
        self.left_wheel.setVelocity(0)
        self.right_wheel.setVelocity(0)
        return True

    def turn_gyro(self, speed=2, direction="left"):
        """
        Rotate the robot 90 degrees to the left or right.
        
        Parameters:
        - robot: instance of the robot.
        - left_wheel: left wheel motor device.
        - right_wheel: right wheel motor device.
        - gyro: gyro sensor.
        - speed: wheel speed during the rotation.
        - direction: direction of the rotation (left or right).
        """
        target_angle = math.radians(90)  # Convertir 90 grados a radianes
        target_cardinal = self.get_target_orientation(direction)
        self.current_orientation = 0

        # Rotate the robot
        if direction == "left":
            self.left_wheel.setVelocity(-speed)
            self.right_wheel.setVelocity(speed)
        else:
            self.left_wheel.setVelocity(speed)
            self.right_wheel.setVelocity(-speed)

        while self.robot.step(TIME_STEP) != -1:
            gyro_values = self.gyro.getValues()
            angular_velocity_rad = gyro_values[2]
            self.current_orientation += angular_velocity_rad * TIME_STEP / 1000

            if abs(self.current_orientation) >= target_angle:
                break

        self.current_cardinal = target_cardinal
        self.left_wheel.setVelocity(0)
        self.right_wheel.setVelocity(0)

    def get_target_orientation(self, direction="left"):
        """
        Get the target orientation of the robot after turning 90 degrees to the left.

        Returns:
        - str: The target orientation of the robot after turning 90 degrees to the left.

        Raises:
        - ValueError: If an invalid orientation is provided.
        """
        if self.current_cardinal not in ("N", "E", "S", "W"):
            raise ValueError("Invalid orientation. Must be 'N', 'E', 'S', or 'W'.")
        
        new_cardinal = None
        for cardinal, _ in self.ORIENTATIONS.items():
            if cardinal == self.current_cardinal:
                new_cardinal = cardinal
                break

        if new_cardinal is None:
            raise ValueError(
                "Current orientation does not match any known cardinal direction."
            )

        # Define turn mapping based on current cardinal direction
        turn_mapping = {
            "N": {"left": "W", "right": "E"},
            "E": {"left": "N", "right": "S"},
            "S": {"left": "E", "right": "W"},
            "W": {"left": "S", "right": "N"},
        }

        target_orientation = turn_mapping[new_cardinal][direction]

        return target_orientation



    def wall_follow(self, speed=10):
        """
        Implements a wall following algorithm to map the environment and navigate along walls.

        Parameters:
        - robot: instance of the robot.
        - left_wheel: left wheel motor device.
        - right_wheel: right wheel motor device.
        - ir_sensor_list: dictionary containing activated infrared sensors.
        - speed: speed of the wheels during navigation.

        Details:
        The function continuously updates an environmental map while following the perimeter of the
        walls. If a wall is detected in front, the robot may turn to avoid it. The loop terminates
        when the robot returns to the start position, indicated by coordinates.
        """
        env_map = create_map()
        self.current_position = [int(MAP_SIZE / 2), int(MAP_SIZE / 2)]

        # Check ir sensors
        while True:
            self.robot.step(TIME_STEP)
            front_ir = self.ir_sensor_list["front infrared sensor"]
            left_ir = self.ir_sensor_list["left infrared sensor"]
            right_ir = self.ir_sensor_list["right infrared sensor"]
            back_ir = self.ir_sensor_list["rear infrared sensor"]
            print("Front IR: ", front_ir.getValue())
            print("Left IR: ", left_ir.getValue())
            print("Right IR: ", right_ir.getValue())

            env_map = update_map(
                env_map,
                self.current_position,
                self.current_cardinal,
                front_ir,
                left_ir,
                right_ir,
                back_ir,
            )

            # If there is a wall in front of the robot, and if the robot turn right is not the start
            # position
            if front_ir.getValue() >= 190 and not left_ir.getValue() <= 160:
                # Turn right
                self.turn_gyro(2, "right")
            elif left_ir.getValue() <= 160:
                # Turn left
                self.turn_gyro(2, "left")

                env_map = update_map(
                    env_map,
                    self.current_position,
                    self.current_cardinal,
                    front_ir,
                    left_ir,
                    right_ir,
                    back_ir,
                )

                if self.move_forward(0.25, speed):
                    self.current_position = self.change_position()
            else:
                if self.move_forward(0.25, speed):
                    self.current_position = self.change_position()

            if self.current_position == [int(MAP_SIZE / 2), int(MAP_SIZE / 2)]:
                print("Start position reached.")
                break

        env_map = fill_map(env_map)
        display_map(env_map)


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
        if self.current_cardinal not in ("N", "E", "S", "W"):
            raise ValueError("Invalid current_cardinal. Must be 'N', 'E', 'S', or 'W'.")

        print("Current cardinal: ", self.current_cardinal)
        if self.current_cardinal == "N":
            self.current_position[0] -= 1  # Move up in the grid (decrease row index)
        elif self.current_cardinal == "E":
            self.current_position[1] += 1  # Move right in the grid (increase column index)
        elif self.current_cardinal == "S":
            self.current_position[0] += 1  # Move down in the grid (increase row index)
        elif self.current_cardinal == "W":
            self.current_position[1] -= 1  # Move left in the grid (decrease column index)

        return self.current_position