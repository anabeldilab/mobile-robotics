"""
Robótica móvil
Master en informática industrial y robótica
Universidade da Coruña
Author: Anabel Díaz Labrador
        Jaime Pablo Pérez Moro

Control of the Khepera IV robot's movement in Webots using Supervisor.
"""

from controller import Supervisor
from .mapping import create_map, display_map, update_map, fill_map, MAP_SIZE

ORIENTATIONS = {
    "N": [[0, -1], [-1, 0]],
    "E": [[1, 0], [0, -1]],
    "S": [[0, 1], [1, 0]],
    "W": [[-1, 0], [0, 1]],
}

# Maximum wheel speed supported by the robot (khepera4).
MAX_SPEED = 47.6
# Default speed for this behavior.
CRUISE_SPEED = 4
# Default time step for the controller.
TIME_STEP = 32

# Names of the infrared-based distance sensors.
INFRARED_SENSORS_NAMES = [
    "rear left infrared sensor",
    "left infrared sensor",
    "front left infrared sensor",
    "front infrared sensor",
    "front right infrared sensor",
    "right infrared sensor",
    "rear right infrared sensor",
    "rear infrared sensor",
]


def enable_distance_sensors(robot, time_step, sensor_names):
    """
    Activate and retrieve distance sensors.

    Parameters:
    - robot: instance of the robot.
    - time_step: update interval for sensors/actuators in milliseconds.
    - sensor_names: list of sensor names to activate.

    Returns:
    - dict: Dictionary with activated distance sensors, ordered as in the sensor_names list.
    """
    sensor_dict = {}

    for name in sensor_names:
        sensor_dict[name] = robot.getDevice(name)
        sensor_dict[name].enable(time_step)

    return sensor_dict


def init_devices(time_step):
    """
    Set up and retrieve necessary devices.

    Parameters:
    - time_step: default update interval in milliseconds for sensors/actuators.

    Returns:
    - tuple: Contains instances of the robot, wheel motors, infrared sensors, position sensors, and
    camera.
    """
    robot = Supervisor()

    left_wheel = robot.getDevice("left wheel motor")
    right_wheel = robot.getDevice("right wheel motor")
    left_wheel.setPosition(float("inf"))
    right_wheel.setPosition(float("inf"))
    left_wheel.setVelocity(0)
    right_wheel.setVelocity(0)

    ir_sensor_list = enable_distance_sensors(robot, time_step, INFRARED_SENSORS_NAMES)

    camera = robot.getDevice("camera")
    camera.enable(time_step * 10)

    pos_l = robot.getDevice("left wheel sensor")
    pos_r = robot.getDevice("right wheel sensor")
    pos_l.enable(time_step)
    pos_r.enable(time_step)

    return robot, left_wheel, right_wheel, ir_sensor_list, pos_l, pos_r, camera


def move_forward(
    robot, left_wheel, right_wheel, distance, front_ir, speed=CRUISE_SPEED
):
    """
    Move the robot forward a specific distance using closed-loop control.

    Parameters:
    - robot: instance of the robot.
    - left_wheel: left wheel motor device.
    - right_wheel: right wheel motor device.
    - distance: distance to move forward in meters.
    - front_ir: front infrared sensor.
    - speed: wheel speed.

    Returns:
    - True if the target distance is reached without obstacles blocking the path.
    - False if an obstacle prevents reaching the target distance.
    """
    # Get the reference to the robot node and ensure it's valid
    khepera_node = robot.getFromDef("Khepera")
    if not khepera_node:
        print("Error: Khepera robot definition not found.")
        return False

    # Get the initial position
    initial_position = khepera_node.getPosition()
    if initial_position is None:
        print("Error: Could not get the initial position.")
        return False

    # Start the motors
    left_wheel.setVelocity(speed)
    right_wheel.setVelocity(speed)

    # Main loop to move the robot towards the target position
    while robot.step(TIME_STEP) != -1:
        current_position = khepera_node.getPosition()
        # Check if the target distance has been reached
        if any(
            abs(current_position[i] - initial_position[i]) >= distance for i in [0, 2]
        ):
            break
        # Check for obstacles
        if front_ir.getValue() >= 250:
            # Check if the robot has covered at least 75% of the distance
            if any(
                abs(current_position[i] - initial_position[i]) >= 0.75 * distance
                for i in [0, 2]
            ):
                break
            return False

    # Stop the motors
    left_wheel.setVelocity(0)
    right_wheel.setVelocity(0)
    return True


def change_position(current_position, orientation):
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
    if not isinstance(current_position, list) or len(current_position) != 2:
        raise ValueError("current_position must be a list of two integers")
    if orientation not in ("N", "E", "S", "W"):
        raise ValueError("Invalid orientation. Must be 'N', 'E', 'S', or 'W'.")

    if orientation == "N":
        current_position[0] -= 1  # Move up in the grid (decrease row index)
    elif orientation == "E":
        current_position[1] += 1  # Move right in the grid (increase column index)
    elif orientation == "S":
        current_position[0] += 1  # Move down in the grid (increase row index)
    elif orientation == "W":
        current_position[1] -= 1  # Move left in the grid (decrease column index)

    return current_position


def get_orientation(khepera_node):
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
        [round(orientation_matrix[6]), round(orientation_matrix[7])],
    ]
    # Calculate the simplified orientation based on the main components
    if simplified_orientation == ORIENTATIONS["N"]:
        simplified_orientation = "N"
    elif simplified_orientation == ORIENTATIONS["E"]:
        simplified_orientation = "E"
    elif simplified_orientation == ORIENTATIONS["S"]:
        simplified_orientation = "S"
    else:
        simplified_orientation = "W"
    return simplified_orientation


def get_target_orientation(khepera_node, direction="left"):
    """
    Get the target orientation of the khepera_node based on a given turn direction (left or right).

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
        [round(orientation_matrix[6]), round(orientation_matrix[7])]
    ]

    # Convert matrix orientation to cardinal direction
    current_cardinal = None
    for cardinal, matrix in ORIENTATIONS.items():
        if matrix == current_orientation:
            current_cardinal = cardinal
            break

    if current_cardinal is None:
        raise ValueError(
            "Current orientation does not match any known cardinal direction."
        )

    # Define turn mapping based on current cardinal direction
    turn_mapping = {
        "N": {"left": ORIENTATIONS["W"], "right": ORIENTATIONS["E"]},
        "E": {"left": ORIENTATIONS["N"], "right": ORIENTATIONS["S"]},
        "S": {"left": ORIENTATIONS["E"], "right": ORIENTATIONS["W"]},
        "W": {"left": ORIENTATIONS["S"], "right": ORIENTATIONS["N"]},
    }

    # Get the target orientation based on the direction
    target_orientation = turn_mapping[current_cardinal][direction]
    return target_orientation


def correct_trajectory(
    robot, left_wheel, right_wheel, left_ir, right_ir, back_ir, speed=CRUISE_SPEED
):
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
    left_ir_value = left_ir.getValue()
    right_ir_value = right_ir.getValue()

    if left_ir_value >= 1000:
        turn(robot, left_wheel, right_wheel, 2, "right")
        left_wheel.setVelocity(speed)
        right_wheel.setVelocity(speed)
        while robot.step(TIME_STEP) != -1:
            if back_ir.getValue() <= 250:
                turn(robot, left_wheel, right_wheel, 2, "left")
                break

    # If the robot is too far from the wall, turn left
    elif right_ir_value >= 1000:
        turn(robot, left_wheel, right_wheel, 2, "left")
        left_wheel.setVelocity(speed)
        right_wheel.setVelocity(speed)
        while robot.step(TIME_STEP) != -1:
            if back_ir.getValue() <= 250:
                turn(robot, left_wheel, right_wheel, 2, "right")
                break


def turn_tolerance(current, target, tolerance):
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


def turn(robot, left_wheel, right_wheel, speed=CRUISE_SPEED, direction="left"):
    """
    Rotate the robot a specified number of degrees.

    Parameters:
    - robot: instance of the robot.
    - left_wheel: left wheel motor device.
    - right_wheel: right wheel motor device.
    - speed: wheel speed during the rotation.
    - direction: direction of the rotation (left or right).
    """

    khepera_node = robot.getFromDef("Khepera")

    target_orientation = get_target_orientation(khepera_node, direction)

    # Rotate the robot
    if direction == "left":
        left_wheel.setVelocity(-speed)
        right_wheel.setVelocity(speed)
    else:
        left_wheel.setVelocity(speed)
        right_wheel.setVelocity(-speed)

    while robot.step(TIME_STEP) != -1:
        current_orientation = khepera_node.getOrientation()
        current_orientation = [
            [round(current_orientation[0], 2), round(current_orientation[1], 2)],
            [round(current_orientation[6], 2), round(current_orientation[7], 2)],
        ]
        if turn_tolerance(current_orientation, target_orientation, 0.01):
            break

    left_wheel.setVelocity(0)
    right_wheel.setVelocity(0)


def wall_follow(robot, left_wheel, right_wheel, ir_sensor_list, speed=CRUISE_SPEED):
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
    khepera_node = robot.getFromDef("Khepera")
    env_map = create_map()
    current_position = [int(MAP_SIZE / 2), int(MAP_SIZE / 2)]
    current_orientation = get_orientation(khepera_node)

    # Check ir sensors
    while True:
        print("Current orientation: ", current_orientation)
        print("Current position: ", current_position)
        robot.step(TIME_STEP)
        front_ir = ir_sensor_list["front infrared sensor"]
        left_ir = ir_sensor_list["left infrared sensor"]
        right_ir = ir_sensor_list["right infrared sensor"]
        back_ir = ir_sensor_list["rear infrared sensor"]
        print("Front IR: ", front_ir.getValue())
        print("Left IR: ", left_ir.getValue())
        print("Right IR: ", right_ir.getValue())

        correct_trajectory(
            robot, left_wheel, right_wheel, left_ir, right_ir, back_ir, speed
        )

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
            turn(robot, left_wheel, right_wheel, 2, "right")
            current_orientation = get_orientation(khepera_node)
        elif left_ir.getValue() <= 160:
            # Turn left
            turn(robot, left_wheel, right_wheel, 2, "left")
            current_orientation = get_orientation(khepera_node)

            env_map = update_map(
                env_map,
                current_position,
                current_orientation,
                front_ir,
                left_ir,
                right_ir,
                back_ir,
            )

            if move_forward(robot, left_wheel, right_wheel, 0.25, front_ir, speed):
                current_position = change_position(
                    current_position, current_orientation
                )
        else:
            if move_forward(robot, left_wheel, right_wheel, 0.25, front_ir, speed):
                current_position = change_position(
                    current_position, current_orientation
                )

        if current_position == [int(MAP_SIZE / 2), int(MAP_SIZE / 2)]:
            print("Start position reached.")
            break

    env_map = fill_map(env_map)
    display_map(env_map)
