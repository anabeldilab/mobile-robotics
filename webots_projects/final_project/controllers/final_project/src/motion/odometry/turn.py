import math

from src.motion.pid import PID

class Turn:
    def __init__(self, robot, devices):
        """
        Initialize the Turn class.

        Parameters:
        - robot: Instance of the robot.
        - devices: Instance of RobotDevices containing the robot's devices.
        """
        self.robot = robot
        self.devices = devices
        self.pid = PID(2, 0.001)

    def execute(self, direction="left"):
        """
        Rotate the robot 90 degrees to the left or right.

        Parameters:
        - direction: Direction of the rotation (left or right).
        - speed: Wheel speed during the rotation.
        """
        target_angle = math.radians(90)
        current_orientation = 0
        #delta = target_angle * self.devices.WHEEL_BASE / self.devices.WHEEL_RADIUS

        while self.robot.step(self.devices.time_step) != -1:
            gyro_values = self.devices.gyro.getValues()
            angular_velocity_rad = gyro_values[2]
            current_orientation += angular_velocity_rad * self.devices.time_step / 1000

            pid_speed = self.pid.compute(target_angle, abs(current_orientation))
            
            if direction == "left":
                self.devices.left_wheel.setVelocity(-pid_speed)
                self.devices.right_wheel.setVelocity(pid_speed)
            else:
                self.devices.left_wheel.setVelocity(pid_speed)
                self.devices.right_wheel.setVelocity(-pid_speed)

            if abs(current_orientation) >= target_angle:
                break

        self.devices.left_wheel.setVelocity(0)
        self.devices.right_wheel.setVelocity(0)
