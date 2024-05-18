import math

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

    def execute(self, direction="left", speed=2):
        """
        Rotate the robot 90 degrees to the left or right.

        Parameters:
        - direction: Direction of the rotation (left or right).
        - speed: Wheel speed during the rotation.
        """
        target_angle = math.radians(90)
        current_orientation = 0

        if direction == "left":
            self.devices.left_wheel.setVelocity(-speed)
            self.devices.right_wheel.setVelocity(speed)
        else:
            self.devices.left_wheel.setVelocity(speed)
            self.devices.right_wheel.setVelocity(-speed)

        while self.robot.step(self.devices.time_step) != -1:
            gyro_values = self.devices.gyro.getValues()
            angular_velocity_rad = gyro_values[2]
            current_orientation += angular_velocity_rad * self.devices.time_step / 1000

            if abs(current_orientation) >= target_angle:
                break

        self.devices.left_wheel.setVelocity(0)
        self.devices.right_wheel.setVelocity(0)