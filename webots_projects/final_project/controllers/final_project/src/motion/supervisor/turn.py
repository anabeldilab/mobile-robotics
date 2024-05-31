from src.motion.supervisor.orientation import Orientation

class Turn:
    def __init__(self, robot):
        """
        Initialize the Turn class.

        Parameters:
        - robot: Instance of the robot.
        - devices: Instance of RobotDevices containing the robot's devices.
        """
        self.robot = robot

    def execute(self, direction="left", speed=2):
        """
        Rotate the robot a specified number of degrees.

        Parameters:
        - direction: Direction of the rotation (left or right).
        - speed: Speed of the wheels during rotation.
        """
        khepera_node = self.robot.robot.getFromDef("Khepera")
        target_orientation = Orientation.get_target_orientation(khepera_node, direction)

        if direction == "left":
            self.robot.devices.left_wheel.setVelocity(-speed)
            self.robot.devices.right_wheel.setVelocity(speed)
        else:
            self.robot.devices.left_wheel.setVelocity(speed)
            self.robot.devices.right_wheel.setVelocity(-speed)

        while self.robot.step() != -1:
            current_orientation = khepera_node.getOrientation()
            current_orientation = [
                [round(current_orientation[0], 2), round(current_orientation[1], 2)],
                [round(current_orientation[3], 2), round(current_orientation[4], 2)],
            ]
            if self.turn_tolerance(current_orientation, target_orientation, 0.01):
                break

        self.robot.current_orientation = Orientation.get_orientation(khepera_node)

        self.robot.devices.left_wheel.setVelocity(0)
        self.robot.devices.right_wheel.setVelocity(0)

    @staticmethod
    def turn_tolerance(current, target, tolerance):
        """
        Check if the current orientation matrix is within the specified tolerance of the target orientation matrix.

        Parameters:
        - current: 2x2 list (matrix) representing the current orientation.
        - target: 2x2 list (matrix) representing the target orientation.
        - tolerance: float, the tolerance for each element in the orientation matrices.

        Returns:
        - bool: True if within tolerance, False otherwise.
        """
        return all(
            abs(current[i][j] - target[i][j]) <= tolerance
            for i in range(2)
            for j in range(2)
        )