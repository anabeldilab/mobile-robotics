"""Move forward and backward functions for the Khepera robot."""

class MoveForward:
    def __init__(self, robot):
        """
        Initialize the MoveForward class.

        Parameters:
        - robot: Instance of the robot.
        - devices: Instance of RobotDevices containing the robot's devices.
        """
        self.robot = robot

    def execute(self, distance=0.25, speed=10.0):
        """
        Move the robot forward a specific distance using closed-loop control.

        Parameters:
        - distance: Distance to move forward in meters.
        - speed: Wheel speed.

        Returns:
        - True if the target distance is reached without obstacles blocking the path.
        - False if an obstacle prevents reaching the target distance.
        """
        khepera_node = self.robot.robot.getFromDef("Khepera")
        if not khepera_node:
            print("Error: Khepera robot definition not found.")
            return False

        initial_position = khepera_node.getPosition()
        if initial_position is None:
            print("Error: Could not get the initial position.")
            return False

        self.robot.devices.left_wheel.setVelocity(speed)
        self.robot.devices.right_wheel.setVelocity(speed)

        while self.robot.step() != -1:
            current_position = khepera_node.getPosition()
            if any(
                abs(current_position[i] - initial_position[i]) >= distance - 0.01
                for i in [0, 1]
            ):
                break

            if self.robot.devices.ir_sensor_list["front infrared sensor"].getValue() >= 250:
                if any(
                    abs(current_position[i] - initial_position[i]) >= 0.75 * distance
                    for i in [0, 1]
                ):
                    break
                return False
        
        self.robot.devices.left_wheel.setVelocity(0)
        self.robot.devices.right_wheel.setVelocity(0)
        self.robot.current_position = self.robot.change_position()
        return True