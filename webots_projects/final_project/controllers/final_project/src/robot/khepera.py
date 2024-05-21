from controller import Robot
from src.robot.robot_devices import RobotDevices

from src.motion.odometry.turn import Turn
from src.motion.odometry.move import MoveForward
from src.navigation.wall_follow import WallFollower
from src.navigation.path_planning import PathPlanning

class Khepera:
    def __init__(self, time_step=32, max_speed=10):
        """
        Initialize the Khepera class.

        Parameters:
        - time_step: Time step for sensor and actuator updates.
        - max_speed: Maximum speed of the robot.
        """
        self.robot = Robot()
        self.devices = RobotDevices(time_step, self.robot)
        self.move_forward = MoveForward(self.robot, self.devices)
        self.turn = Turn(self.robot, self.devices)
        self.wall_follower = WallFollower(self.robot, self.devices, self.move_forward, self.turn)

    def run(self):
        """Run the wall following behavior."""
        env_map, start_position, goal_position, current_cardinal= self.wall_follower.follow_wall()

        start = (start_position[0], start_position[1])
        goal = (goal_position[0], goal_position[1])

        path_planning = PathPlanning(self.robot, self.devices, self.move_forward, self.turn, env_map)
        print(f"Start: {start}, Goal: {goal}")
        print(f"Map: {env_map}")
        print(f"Path: {path_planning.find_shortest_path(start, goal)}")
        path_planning.execute(start, goal, current_cardinal)
