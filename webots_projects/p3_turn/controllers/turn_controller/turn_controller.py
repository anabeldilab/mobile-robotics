"""turn_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import numpy as np
from controller import Supervisor


# Constants
# Maximum wheel speed supported by the robot (khepera4).
MAX_SPEED = 47.6
# Default speed for this behavior.
CRUISE_SPEED = MAX_SPEED * 0.8
# Default time step for the controller.
TIME_STEP = 32

# create the Robot instance.
robot = Supervisor()
robot_node = robot.getFromDef("Khepera")

if robot_node is None:
    print("No Khepera node found in the current world.")
    exit(1)

trans_field = robot_node.getField("translation")
rot_field = robot_node.getField("rotation")

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Rotate 90 degrees around the z-axis
ANGLE = math.pi / 2  # 90 degrees in radians
ROT = [-1, 0, 0, ANGLE]
INITIAL = [0.125, 0, 0.125]

trans_field.setSFVec3f(INITIAL)
rot_field.setSFRotation(ROT)
robot_node.resetPhysics()


def turn(robot, left_wheel, right_wheel, speed=CRUISE_SPEED, direction="left"):
    """
    Rotate the robot a specified number of degrees.

    Parameters:
    - robot: instance of the robot.
    - left_wheel: left wheel motor device.
    - right_wheel: right wheel motor device.
    - speed: wheel speed during the rotation.
    - direction: direction of the rotation (left or right).
    """

    khepera_node = robot.getFromDef("Khepera")

    left_wheel.setVelocity(0)
    right_wheel.setVelocity(0)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    goal = [0.375, 0, -0.125]
    current_position = robot_node.getField("translation").getSFVec3f()
    rot_field = robot_node.getField("rotation").getSFRotation()
    # Compute differences in x and z (assuming y is the height/vertical axis and ignoring it).
    dx = goal[0] - current_position[0]
    dz = goal[2] - current_position[2]

    angle = np.arctan2(dz, dx)

    
    # Convert the angle to degrees.
    angle = np.degrees(angle)
    print("Angle: ", angle)
