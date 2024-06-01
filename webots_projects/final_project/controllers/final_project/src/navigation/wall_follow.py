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
            sensor_readings, _ = self.robot.collect_sensor_data()

            if sensor_readings["front"] >= 190 and not sensor_readings["left"] <= 160:
                self.robot.turn.execute(direction="right")
            elif sensor_readings["left"] <= 160:
                self.robot.turn.execute(direction="left")
                self.robot.update_map()
                self.robot.move_forward.execute(0.25)
            else:
                self.robot.move_forward.execute(0.25)
            print(f"current_position: {self.robot.current_position}")
            print(f"start_position: {self.robot.start_position}")
            if self.robot.current_position == self.robot.start_position:
                print("Start position reached.")
                break

    def start_find_wall(self):
        """Find the wall and align the robot to it the first time it starts moving."""
        if self.robot.devices.ir_sensor_list["front infrared sensor"].getValue() >= 190:
            self.robot.turn.execute(direction="right")
        elif self.robot.devices.ir_sensor_list["right infrared sensor"].getValue() >= 190:
            self.robot.turn.execute(direction="left")
