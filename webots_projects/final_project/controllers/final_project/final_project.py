"""
Robótica móvil
Master en informática industrial y robótica
Universidade da Coruña
Author: Anabel Díaz Labrador
        Jaime Pablo Pérez Moro

Control del robot Khepera IV en Webots usando Supervisor.
"""

from src.robot.khepera_supervisor import KheperaSupervisor
from src.robot.khepera import Khepera


def main():
    """
    Función principal para controlar el robot.
    """
    #khepera_supervisor = KheperaSupervisor()
    #khepera_supervisor.run()

    khepera = Khepera()
    khepera.run()


if __name__ == "__main__":
    main()
