from controller import Supervisor
from src.robot.robot_devices import RobotDevices

from src.motion.supervisor.turn import Turn
from src.motion.supervisor.move import MoveForward
from src.navigation.wall_follow import WallFollowerSupervisor

class KheperaSupervisor:
    def __init__(self, time_step=32, max_speed=47.6):
        """
        Initialize the KheperaSupervisor class.

        Parameters:
        - time_step: Time step for sensor and actuator updates.
        - max_speed: Maximum speed of the robot.
        """
        self.robot = Supervisor()
        self.devices = RobotDevices(time_step, self.robot)
        self.move_forward = MoveForward(self.robot, self.devices)
        self.turn = Turn(self.robot, self.devices)
        self.wall_follower = WallFollowerSupervisor(self.robot, self.move_forward, self.turn, self.devices)

    def run(self):
        """Run the wall following behavior."""
        self.wall_follower.follow_wall()
        # trajectory

