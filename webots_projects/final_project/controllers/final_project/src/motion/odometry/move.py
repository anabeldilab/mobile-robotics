from src.motion.pid import PID


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
        self.pid = PID(1, 0.0001, 0.000001)

    def execute(self, distance=0.25):
        """
        Move the robot forward a specific distance using odometry.

        Parameters:
        - distance: Distance to move forward in meters.
        - speed: Wheel speed.
        """
        delta_position = distance / self.devices.WHEEL_RADIUS
        initial_pos_l = self.devices.pos_l.getValue()
        initial_pos_r = self.devices.pos_r.getValue()

        # Check if initial positions are floats
        while not isinstance(initial_pos_l, float) or not isinstance(
            initial_pos_r, float
        ):
            self.robot.step(self.devices.time_step)
            initial_pos_l = self.devices.pos_l.getValue()
            initial_pos_r = self.devices.pos_r.getValue()

        target_pos_l = initial_pos_l + delta_position
        target_pos_r = initial_pos_r + delta_position

        while self.robot.step(self.devices.time_step) != -1:
            # Set target position

            current_pos_l = self.devices.pos_l.getValue()
            current_pos_r = self.devices.pos_r.getValue()

            pid_speed_l = self.pid.compute(target_pos_l, current_pos_l)
            pid_speed_r = self.pid.compute(target_pos_r, current_pos_r)

            self.devices.left_wheel.setVelocity(pid_speed_l)
            self.devices.right_wheel.setVelocity(pid_speed_r)

            # Check if the target position has been reached
            if (
                current_pos_l >= target_pos_l
                and current_pos_r >= target_pos_r
            ):
                break

        self.devices.left_wheel.setVelocity(0)
        self.devices.right_wheel.setVelocity(0)
        return True
