"""
Robótica móvil
Master en informática industrial y robótica
Universidade da Coruña
Author: Anabel Díaz Labrador
        Jaime Pablo Pérez Moro

Control del robot Khepera IV en Webots usando Supervisor.
"""

from math import fabs, pi, atan2
from controller import Supervisor


# Máxima velocidad de las ruedas soportada por el robot (khepera4).
MAX_SPEED = 47.6
# Velocidad por defecto para este comportamiento.
CRUISE_SPEED = 8
# Time step por defecto para el controlador.
TIME_STEP = 32

# Nombres de los sensores de distancia basados en infrarrojo.
INFRARED_SENSORS_NAMES = [
    "rear left infrared sensor",
    "left infrared sensor",
    "front left infrared sensor",
    "front infrared sensor",
    "front right infrared sensor",
    "right infrared sensor",
    "rear right infrared sensor",
    "rear infrared sensor",
]


def enable_distance_sensors(robot, timeStep, sensorNames):
    """
    Obtener y activar los sensores de distancia.

    Return: lista con los sensores de distancia activados, en el mismo orden
    establecido en la lista de  nombres (sensorNames).
    """

    sensorList = []

    for name in sensorNames:
        sensorList.append(robot.getDevice(name))

    for sensor in sensorList:
        sensor.enable(timeStep)

    return sensorList


def init_devices(timeStep):
    """
    Obtener y configurar los dispositivos necesarios.

    timeStep: tiempo (en milisegundos) de actualización por defecto para los sensores/actuadores
      (cada dispositivo puede tener un valor diferente).
    """

    # Get pointer to the robot.
    robot = Supervisor()

    # Si queremos obtener el timestep de la simulación.
    # simTimeStep = int(robot.getBasicTimeStep())

    # Obtener dispositivos correspondientes a los motores de las ruedas.
    leftWheel = robot.getDevice("left wheel motor")
    rightWheel = robot.getDevice("right wheel motor")

    # Configuración inicial para utilizar movimiento por posición (necesario para odometría).
    # En movimiento por velocidad, establecer posición a infinito (wheel.setPosition(float('inf'))).
    leftWheel.setPosition(float("inf"))
    rightWheel.setPosition(float("inf"))
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)

    # Obtener una lista con los sensores infrarrojos ya activados
    irSensorList = enable_distance_sensors(robot, timeStep, INFRARED_SENSORS_NAMES)

    # Obtener el dispositivo de la cámara
    camera = robot.getDevice("camera")
    # Activar el dispositivo de la cámara (el tiempo de actualización de los frames
    # de la cámara no debería ser muy alto debido al alto volumen de datos que implica).
    camera.enable(timeStep * 10)

    # Obtener y activar los sensores de posición de las ruedas (encoders).
    posL = robot.getDevice("left wheel sensor")
    posR = robot.getDevice("right wheel sensor")
    posL.enable(timeStep)
    posR.enable(timeStep)

    # TODO: Obtener y activar otros dispositivos necesarios.
    # ...

    return robot, leftWheel, rightWheel, irSensorList, posL, posR, camera


def move_forward(robot, leftWheel, rightWheel, distance, speed=CRUISE_SPEED):
    """
    Mover el robot hacia adelante una distancia específica usando control supervisado.

    robot: instancia del robot.
    leftWheel, rightWheel: dispositivos de los motores de las ruedas.
    distance: distancia a moverse hacia adelante en metros.
    speed: velocidad de las ruedas.
    """
    # Obtener la posición inicial del robot
    khepera_node = robot.getFromDef("Khepera")
    initial_position = khepera_node.getPosition()

    # Activar los motores para mover el robot hacia adelante
    leftWheel.setVelocity(speed)
    rightWheel.setVelocity(speed)
    print("Distance: " + str(distance))

    # Bucle principal para mover el robot hacia la posición objetivo
    while robot.step(TIME_STEP) != -1:
        current_position = khepera_node.getPosition()
        print("Current position: " + str(current_position))
        if current_position[2] >= initial_position[2] + distance: # Down
            break
        if current_position[0] >= initial_position[0] + distance: # Right
            break
        if current_position[0] <= initial_position[0] - distance: # Left
            break
        if current_position[2] <= initial_position[2] - distance: # Up
            break
    # Detener los motores
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)
    print("Movimiento completado hacia adelante por", distance, "metros.")


def turn_left(robot, leftWheel, rightWheel, degrees, speed=CRUISE_SPEED):
    """
    Gira el robot una cantidad específica de grados a una velocidad dada.
    
    robot: instancia del robot Supervisor.
    leftWheel, rightWheel: motores de las ruedas izquierda y derecha.
    degrees: cantidad de grados para girar. Positivo para girar a la izquierda, negativo para la derecha.
    speed: velocidad de las ruedas durante el giro.
    """
    # Convertir grados a radianes
    target_rotation_radians = degrees * (pi / 180)

    # Iniciar el giro
    if degrees > 0:  # Girar a la izquierda
        leftWheel.setVelocity(-speed)
        rightWheel.setVelocity(speed)
    else:  # Girar a la derecha
        leftWheel.setVelocity(speed)
        rightWheel.setVelocity(-speed)

    # Obtener el nodo del robot y su rotación inicial
    khepera_node = robot.getFromDef('Khepera')
    initial_rotation = khepera_node.getOrientation()
    initial_angle = atan2(initial_rotation[6], initial_rotation[0])  # atan2(rot_z, rot_x)

    # Ángulo objetivo en radianes
    target_angle = initial_angle + target_rotation_radians

    # Normalizar el ángulo objetivo para mantenerlo entre -pi y pi
    if target_angle > pi:
        target_angle -= 2 * pi
    elif target_angle < -pi:
        target_angle += 2 * pi

    # Bucle de giro
    while robot.step(TIME_STEP) != -1:
        current_rotation = khepera_node.getOrientation()
        current_angle = atan2(current_rotation[6], current_rotation[0])

        # Normalizar el ángulo actual
        if current_angle > pi:
            current_angle -= 2 * pi
        elif current_angle < -pi:
            current_angle += 2 * pi

        # Diferencia de ángulo
        angle_diff = current_angle - target_angle

        # Normalizar la diferencia de ángulo
        if angle_diff > pi:
            angle_diff -= 2 * pi
        elif angle_diff < -pi:
            angle_diff += 2 * pi

        # Comprobar si el giro se completó dentro de un umbral
        if fabs(angle_diff) < 0.03:  # 0.05 radianes de margen de error
            break

    # Detener las ruedas
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)
    print(f"Giro completado de {degrees} grados.")


def main():
    # Activamos los dispositivos necesarios y obtenemos referencias a ellos.
    robot, leftWheel, rightWheel, irSensorList, posL, posR, camera = init_devices(
        TIME_STEP
    )

    # TODO Implementar arquitectura de control del robot.
    # ...

    move_forward(robot, leftWheel, rightWheel, 0.25)
    turn_left(robot, leftWheel, rightWheel, 90)
    move_forward(robot, leftWheel, rightWheel, 0.25)


if __name__ == "__main__":
    main()
