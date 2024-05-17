from controller import Supervisor
from src.motion.movement import create_map, MAP_SIZE, update_map, change_position
from src.map.mapping import fill_map, display_map


class KheperaSupervisor:
    """Khepera robot class that uses odometry and gyroscope to know its position"""

    def __init__(self, time_step=32, max_speed=47.6):
        self.time_step = time_step
        (
            self.robot,
            self.left_wheel,
            self.right_wheel,
            self.ir_sensor_list,
            self.camera,
        ) = self.init_devices()

    ORIENTATIONS = {
        "N": [[0, -1], [1, 0]],
        "E": [[1, 0], [0, 1]],
        "S": [[0, 1], [-1, 0]],
        "W": [[-1, 0], [0, -1]],
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
        self.robot = Supervisor()

        self.left_wheel = self.robot.getDevice("left wheel motor")
        self.right_wheel = self.robot.getDevice("right wheel motor")
        self.left_wheel.setPosition(float("inf"))
        self.right_wheel.setPosition(float("inf"))
        self.left_wheel.setVelocity(0)
        self.right_wheel.setVelocity(0)

        ir_sensor_list = self.enable_distance_sensors()

        camera = self.robot.getDevice("camera")
        camera.enable(self.time_step * 10)

        return self.robot, self.left_wheel, self.right_wheel, ir_sensor_list, camera

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

    def move_forward(self, distance, speed=10.0):
        """
        Move the robot forward a specific distance using closed-loop control.

        Parameters:
        - distance: distance to move forward in meters.
        - speed: wheel speed.

        Returns:
        - True if the target distance is reached without obstacles blocking the path.
        - False if an obstacle prevents reaching the target distance.
        """
        # Get the reference to the robot node and ensure it's valid
        khepera_node = self.robot.getFromDef("Khepera")
        if not khepera_node:
            print("Error: Khepera robot definition not found.")
            return False

        # Get the initial position
        initial_position = khepera_node.getPosition()
        if initial_position is None:
            print("Error: Could not get the initial position.")
            return False

        # Start the motors
        self.left_wheel.setVelocity(speed)
        self.right_wheel.setVelocity(speed)

        # Main loop to move the robot towards the target position
        while self.robot.step(self.time_step) != -1:
            print("Current position: ", khepera_node.getPosition())
            print("Initial position: ", initial_position)
            print("Distance: ", distance)
            current_position = khepera_node.getPosition()
            # Check if the target distance has been reached
            if any(
                abs(current_position[i] - initial_position[i]) >= distance - 0.01
                for i in [0, 1]
            ):
                break
            # Check for obstacles

            if self.ir_sensor_list["front infrared sensor"].getValue() >= 250:
                # Check if the robot has covered at least 75% of the distance
                if any(
                    abs(current_position[i] - initial_position[i]) >= 0.75 * distance
                    for i in [0, 1]
                ):
                    break
                return False

        # Stop the motors
        self.left_wheel.setVelocity(0)
        self.right_wheel.setVelocity(0)
        return True

    def turn(self, speed=10, direction="left"):
        """
        Rotate the robot a specified number of degrees.

        Parameters:
        - direction: direction of the rotation (left or right).
        """

        khepera_node = self.robot.getFromDef("Khepera")

        target_orientation = self.get_target_orientation(khepera_node, direction)

        # Rotate the robot
        if direction == "left":
            self.left_wheel.setVelocity(-speed)
            self.right_wheel.setVelocity(speed)
        else:
            self.left_wheel.setVelocity(speed)
            self.right_wheel.setVelocity(-speed)

        while self.robot.step(self.time_step) != -1:
            current_orientation = khepera_node.getOrientation()
            current_orientation = [
                [round(current_orientation[0], 2), round(current_orientation[1], 2)],
                [round(current_orientation[3], 2), round(current_orientation[4], 2)],
            ]
            if self.turn_tolerance(current_orientation, target_orientation, 0.01):
                break

        self.left_wheel.setVelocity(0)
        self.right_wheel.setVelocity(0)

    def get_target_orientation(self, khepera_node, direction="left"):
        """
        Get the target orientation of the khepera_node based on a given turn direction (left or
        right).

        Parameters:
        - khepera_node: The node object representing the Khepera robot.
        - direction (str): The turn direction ('left' or 'right'). Defaults to 'left'.

        Returns:
        - list of list of int: The target orientation matrix for the given turn direction.
        """
        # Get the initial orientation matrix from the node
        orientation_matrix = khepera_node.getOrientation()
        # Simplify the matrix to the main components for comparison
        current_orientation = [
            [round(orientation_matrix[0]), round(orientation_matrix[1])],
            [round(orientation_matrix[3]), round(orientation_matrix[4])],
        ]

        # Convert matrix orientation to cardinal direction
        current_cardinal = None
        for cardinal, matrix in self.ORIENTATIONS.items():
            print("Cardinal: ", )
            print("Matrix: ", matrix)
            print("Current orientation: ", current_orientation)
            if matrix == current_orientation:
                current_cardinal = cardinal
                break

        if current_cardinal is None:
            raise ValueError(
                "Current orientation does not match any known cardinal direction."
            )

        # Define turn mapping based on current cardinal direction
        turn_mapping = {
            "N": {"left": self.ORIENTATIONS["W"], "right": self.ORIENTATIONS["E"]},
            "E": {"left": self.ORIENTATIONS["N"], "right": self.ORIENTATIONS["S"]},
            "S": {"left": self.ORIENTATIONS["E"], "right": self.ORIENTATIONS["W"]},
            "W": {"left": self.ORIENTATIONS["S"], "right": self.ORIENTATIONS["N"]},
        }

        # Get the target orientation based on the direction
        target_orientation = turn_mapping[current_cardinal][direction]
        return target_orientation

    def turn_tolerance(self, current, target, tolerance):
        """
        Check if the current orientation matrix is within the specified tolerance of the target
        orientation matrix.

        Parameters:
        - current: 2x2 list (matrix) representing the current orientation.
        - target: 2x2 list (matrix) representing the target orientation.
        - tolerance: float, the tolerance for each element in the orientation matrices.
        :return: bool, True if within tolerance, False otherwise.
        """
        return all(
            abs(current[i][j] - target[i][j]) <= tolerance
            for i in range(2)
            for j in range(2)
        )


    def correct_trajectory(self, speed=10):
        """
        Correct the trajectory of the robot based on the IR sensors.

        Parameters:
        - robot: instance of the robot.
        - left_wheel, right_wheel: wheel motors.
        - left_ir: left infrared sensor.
        - right_ir: right infrared sensor.
        - back_ir: back infrared sensor.
        - speed: speed of the wheels.
        """
        # If the robot is too close to the wall, turn right
        left_ir = self.ir_sensor_list["left infrared sensor"]
        right_ir = self.ir_sensor_list["right infrared sensor"]
        back_ir = self.ir_sensor_list["rear infrared sensor"]

        left_ir_value = left_ir.getValue()
        right_ir_value = right_ir.getValue()

        if left_ir_value >= 1000:
            self.turn(2, "right")
            self.left_wheel.setVelocity(speed)
            self.right_wheel.setVelocity(speed)
            while self.robot.step(self.time_step) != -1:
                if back_ir.getValue() <= 250:
                    self.turn(2, "left")
                    break

        # If the robot is too far from the wall, turn left
        elif right_ir_value >= 1000:
            self.turn(2, "left")
            self.left_wheel.setVelocity(speed)
            self.right_wheel.setVelocity(speed)
            while self.robot.step(self.time_step) != -1:
                if back_ir.getValue() <= 250:
                    self.turn(2, "right")
                    break

    def get_orientation(self, khepera_node):
        """
        Get the orientation of the khepera_node based on its orientation matrix.

        Parameters:
        - khepera_node: The node object representing the Khepera robot.

        Returns:
        - str: The cardinal orientation of the node ('N', 'E', 'S', 'W').
        """
        orientation_matrix = khepera_node.getOrientation()
        simplified_orientation = [
            [round(orientation_matrix[0]), round(orientation_matrix[1])],
            [round(orientation_matrix[3]), round(orientation_matrix[4])],
        ]
        # Calculate the simplified orientation based on the main components
        if simplified_orientation == self.ORIENTATIONS["N"]:
            simplified_orientation = "N"
        elif simplified_orientation == self.ORIENTATIONS["E"]:
            simplified_orientation = "E"
        elif simplified_orientation == self.ORIENTATIONS["S"]:
            simplified_orientation = "S"
        else:
            simplified_orientation = "W"
        return simplified_orientation


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
        khepera_node = self.robot.getFromDef("Khepera")
        env_map = create_map()
        current_position = [int(MAP_SIZE / 2), int(MAP_SIZE / 2)]
        current_orientation = self.get_orientation(khepera_node)

        # Check ir sensors
        while True:
            print("Current orientation: ", current_orientation)
            print("Current position: ", current_position)
            self.robot.step(self.time_step)
            front_ir = self.ir_sensor_list["front infrared sensor"]
            left_ir = self.ir_sensor_list["left infrared sensor"]
            right_ir = self.ir_sensor_list["right infrared sensor"]
            back_ir = self.ir_sensor_list["rear infrared sensor"]
            print("Front IR: ", front_ir.getValue())
            print("Left IR: ", left_ir.getValue())
            print("Right IR: ", right_ir.getValue())

            self.correct_trajectory(speed)

            env_map = update_map(
                env_map,
                current_position,
                current_orientation,
                front_ir,
                left_ir,
                right_ir,
                back_ir,
            )

            # If there is a wall in front of the robot, and if the robot turn right is not the start
            # position
            if front_ir.getValue() >= 190 and not left_ir.getValue() <= 160:
                # Turn right
                self.turn(2, "right")
                current_orientation = self.get_orientation(khepera_node)
            elif left_ir.getValue() <= 160:
                # Turn left
                self.turn(2, "left")
                current_orientation = self.get_orientation(khepera_node)

                env_map = update_map(
                    env_map,
                    current_position,
                    current_orientation,
                    front_ir,
                    left_ir,
                    right_ir,
                    back_ir,
                )

                if self.move_forward(0.25, speed):
                    current_position = change_position(
                        current_position, current_orientation
                    )
            else:
                if self.move_forward(0.25, speed):
                    current_position = change_position(
                        current_position, current_orientation
                    )

            if current_position == [int(MAP_SIZE / 2), int(MAP_SIZE / 2)]:
                print("Start position reached.")
                break

        env_map = fill_map(env_map)
        display_map(env_map)
