"""
Robótica móvil
Master en informática industrial y robótica
Universidade da Coruña
Author: Anabel Díaz Labrador
        Jaime Pablo Pérez Moro

SLAM for the robot Khepera IV in Webots
"""

from src.robot.khepera_supervisor import KheperaSupervisor
from src.robot.khepera_odometry import KheperaOdometry


def main():
    """
    Main function to run the Khepera robot.
    """
    #khepera_supervisor = KheperaSupervisor()
    #khepera_supervisor.run()

    khepera = KheperaOdometry()
    khepera.run()


if __name__ == "__main__":
    main()
