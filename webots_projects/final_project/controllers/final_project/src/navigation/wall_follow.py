class WallFollowerSupervisor:
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
        self.find_wall()

        while True:
            self.robot.step(self.devices.time_step)
            front_ir = self.devices.ir_sensor_list["front infrared sensor"]
            left_ir = self.devices.ir_sensor_list["left infrared sensor"]

            self.correct_trajectory(speed)

            self.robot.mapping.update_map()

            if front_ir.getValue() >= 190 and not left_ir.getValue() <= 160:
                self.turn.execute(direction="right", speed=2)
            elif left_ir.getValue() <= 160:
                self.turn.execute(direction="left", speed=2)
                self.robot.mapping.update_map()

                if self.move_forward.execute(0.25, speed):
                    current_position = self.robot.change_position()
            else:
                if self.move_forward.execute(0.25, speed):
                    current_position = self.robot.change_position()

            if current_position == self.robot.start_position:
                break

        self.robot.mapping.fill_map()
        self.robot.mapping.display_map()

    def find_wall(self):
        """Find the wall and align the robot to it the first time it starts moving."""
        if self.devices.ir_sensor_list["front infrared sensor"].getValue() >= 190:
            self.turn.execute(direction="right")
        elif self.devices.ir_sensor_list["right infrared sensor"].getValue() >= 190:
            self.turn.execute(direction="left")


class WallFollower:
    def __init__(self, robot):
        """
        Initialize the WallFollower class.

        Parameters:
        - robot: Instance of the robot.
        """
        self.robot = robot

    def follow_wall(self):
        """
        Implements a wall following algorithm to map the environment and navigate along walls.

        Parameters:
        - speed: Speed of the wheels during navigation.
        """
        self.start_find_wall()

        while True:
            self.robot.update_map()
            sensor_readings, front_is_yellow = self.robot.collect_sensor_data()

            if sensor_readings["front"] >= 190 and not sensor_readings["left"] <= 160:
                self.robot.turn.execute(direction="right")
            elif sensor_readings["left"] <= 160:
                self.robot.turn.execute(direction="left")
                self.robot.update_map()
                if self.robot.move_forward.execute(0.25):
                    self.robot.current_position = self.robot.change_position()
            else:
                if self.robot.move_forward.execute(0.25):
                    self.robot.current_position = self.robot.change_position()
            print(f"current_position: {self.robot.current_position}")
            print(f"start_position: {self.robot.start_position}")
            if self.robot.current_position == self.robot.start_position:
                print("Start position reached.")
                break

            print(f"goal_position: {self.robot.goal_position}")

    def start_find_wall(self):
        """Find the wall and align the robot to it the first time it starts moving."""
        print(f"{self.robot.devices.ir_sensor_list}")
        if self.robot.devices.ir_sensor_list["front infrared sensor"].getValue() >= 190:
            self.robot.turn.execute(direction="right")
        elif self.robot.devices.ir_sensor_list["right infrared sensor"].getValue() >= 190:
            self.robot.turn.execute(direction="left")

