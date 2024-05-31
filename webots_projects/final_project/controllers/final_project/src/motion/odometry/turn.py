import math

from src.motion.pid import PID
from src.motion.odometry.orientation import Orientation


class Turn:
    def __init__(self, robot):
        """
        Initialize the Turn class.

        Parameters:
        - robot: Instance of the robot.
        """
        self.robot = robot
        self.pid = PID(1.5, 0.001)

    def execute(self, direction="left"):
        """
        Rotate the robot 90 degrees to the left or right.

        Parameters:
        - direction: Direction of the rotation (left or right).
        - speed: Wheel speed during the rotation.
        """
        target_angle = math.radians(90)
        current_orientation = 0
        # delta = target_angle * self.robot.devices.WHEEL_BASE / self.robot.devices.WHEEL_RADIUS

        while self.robot.step() != -1:
            gyro_values = self.robot.devices.gyro.getValues()
            angular_velocity_rad = gyro_values[2]
            current_orientation += angular_velocity_rad * self.robot.devices.time_step / 1000

            pid_speed = self.pid.compute(target_angle, abs(current_orientation))

            if direction == "left":
                self.robot.devices.left_wheel.setVelocity(-pid_speed)
                self.robot.devices.right_wheel.setVelocity(pid_speed)
            else:
                self.robot.devices.left_wheel.setVelocity(pid_speed)
                self.robot.devices.right_wheel.setVelocity(-pid_speed)

            if abs(current_orientation) >= target_angle:
                break

        self.robot.current_cardinal = Orientation.get_target_orientation(
            self.robot.current_cardinal, direction
        )

        self.robot.devices.left_wheel.setVelocity(0)
        self.robot.devices.right_wheel.setVelocity(0)
