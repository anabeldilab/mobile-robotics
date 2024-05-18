class MoveForward:
    def __init__(self, robot, devices):
        """
        Initialize the MoveForward class.

        Parameters:
        - robot: Instance of the robot.
        - devices: Instance of RobotDevices containing the robot's devices.
        """
        self.robot = robot
        self.devices = devices

    def execute(self, distance=0.25, speed=10):
        """
        Move the robot forward a specific distance using odometry.

        Parameters:
        - distance: Distance to move forward in meters.
        - speed: Wheel speed.
        """
        delta_position = distance / self.devices.WHEEL_RADIUS
        initial_pos_l = self.devices.pos_l.getValue()
        initial_pos_r = self.devices.pos_r.getValue()

        if initial_pos_l is not float or initial_pos_r is not float:
            initial_pos_l = self.devices.pos_l.getValue()
            initial_pos_r = self.devices.pos_r.getValue()

        self.devices.left_wheel.setVelocity(speed)
        self.devices.right_wheel.setVelocity(speed)

        while self.robot.step(self.devices.time_step) != -1:
            current_pos_l = self.devices.pos_l.getValue()
            current_pos_r = self.devices.pos_r.getValue()
            left_distance = (current_pos_l - initial_pos_l) * self.devices.WHEEL_RADIUS
            right_distance = (current_pos_r - initial_pos_r) * self.devices.WHEEL_RADIUS

            if (left_distance + right_distance) / 2.0 >= distance:
                break

        self.devices.left_wheel.setVelocity(0)
        self.devices.right_wheel.setVelocity(0)
        return True