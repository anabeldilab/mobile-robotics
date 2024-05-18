from src.motion.movement import create_map, MAP_SIZE, update_map, change_position
from src.map.mapping import fill_map, display_map

from src.motion.supervisor.orientation import Orientation

class WallFollower:
    def __init__(self, robot, move_forward, turn, devices):
        """
        Initialize the WallFollower class.

        Parameters:
        - robot: Instance of the robot.
        - move_forward: Instance of MoveForward class for handling forward movement.
        - turn: Instance of Turn class for handling turns.
        - devices: Instance of RobotDevices containing the robot's devices.
        """
        self.robot = robot
        self.move_forward = move_forward
        self.turn = turn
        self.devices = devices

    def correct_trajectory(self, speed=10):
        """
        Correct the trajectory of the robot based on the IR sensors.

        Parameters:
        - speed: Speed of the wheels.
        """
        left_ir = self.devices.ir_sensor_list["left infrared sensor"]
        right_ir = self.devices.ir_sensor_list["right infrared sensor"]
        back_ir = self.devices.ir_sensor_list["rear infrared sensor"]

        left_ir_value = left_ir.getValue()
        right_ir_value = right_ir.getValue()

        if left_ir_value >= 1000:
            self.turn.execute(direction="right", speed=2)
            self.devices.left_wheel.setVelocity(speed)
            self.devices.right_wheel.setVelocity(speed)
            while self.robot.step(self.devices.time_step) != -1:
                if back_ir.getValue() <= 250:
                    self.turn.execute(direction="left", speed=2)
                    break

        elif right_ir_value >= 1000:
            self.turn.execute(direction="left", speed=2)
            self.devices.left_wheel.setVelocity(speed)
            self.devices.right_wheel.setVelocity(speed)
            while self.robot.step(self.devices.time_step) != -1:
                if back_ir.getValue() <= 250:
                    self.turn.execute(direction="right", speed=2)
                    break

    def follow_wall(self, speed=10):
        """
        Implements a wall following algorithm to map the environment and navigate along walls.

        Parameters:
        - speed: Speed of the wheels during navigation.

        Details:
        The function continuously updates an environmental map while following the perimeter of the
        walls. If a wall is detected in front, the robot may turn to avoid it. The loop terminates
        when the robot returns to the start position, indicated by coordinates.
        """
        khepera_node = self.robot.getFromDef("Khepera")
        env_map = create_map()
        current_position = [int(MAP_SIZE / 2), int(MAP_SIZE / 2)]
        current_orientation = Orientation.get_orientation(khepera_node)

        while True:
            self.robot.step(self.devices.time_step)
            front_ir = self.devices.ir_sensor_list["front infrared sensor"]
            left_ir = self.devices.ir_sensor_list["left infrared sensor"]
            right_ir = self.devices.ir_sensor_list["right infrared sensor"]
            back_ir = self.devices.ir_sensor_list["rear infrared sensor"]

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

            if front_ir.getValue() >= 190 and not left_ir.getValue() <= 160:
                self.turn.execute(direction="right", speed=2)
                current_orientation = Orientation.get_orientation(khepera_node)
            elif left_ir.getValue() <= 160:
                self.turn.execute(direction="left", speed=2)
                current_orientation = Orientation.get_orientation(khepera_node)

                env_map = update_map(
                    env_map,
                    current_position,
                    current_orientation,
                    front_ir,
                    left_ir,
                    right_ir,
                    back_ir,
                )

                if self.move_forward.execute(0.25, speed):
                    current_position = change_position(
                        current_position, current_orientation
                    )
            else:
                if self.move_forward.execute(0.25, speed):
                    current_position = change_position(
                        current_position, current_orientation
                    )

            if current_position == [int(MAP_SIZE / 2), int(MAP_SIZE / 2)]:
                break

        env_map = fill_map(env_map)
        display_map(env_map)