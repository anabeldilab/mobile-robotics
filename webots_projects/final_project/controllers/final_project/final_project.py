"""
Robótica móvil
Master en informática industrial y robótica
Universidade da Coruña
Author: Anabel Díaz Labrador
        Jaime Pablo Pérez Moro

Control del robot Khepera IV en Webots usando Supervisor.
"""

from src.movement import init_devices_robot, wall_follow, TIME_STEP, turn_gyro, move_forward


def main():
    """
    Función principal para controlar el robot.
    """
    # Activamos los dispositivos necesarios y obtenemos referencias a ellos.
    robot, left_wheel, right_wheel, ir_sensor_list, _, _, camera, gyro = init_devices_robot(
        TIME_STEP
    )

    # wall follow
    #wall_follow(robot, left_wheel, right_wheel, ir_sensor_list)
    turn_gyro(robot, left_wheel, right_wheel, gyro, 1)
    turn_gyro(robot, left_wheel, right_wheel, gyro, 1)
    turn_gyro(robot, left_wheel, right_wheel, gyro, 1)
    turn_gyro(robot, left_wheel, right_wheel, gyro, 1, direction="right")
    turn_gyro(robot, left_wheel, right_wheel, gyro, 1, direction="right")
    turn_gyro(robot, left_wheel, right_wheel, gyro, 1, direction="right")



if __name__ == "__main__":
    main()
