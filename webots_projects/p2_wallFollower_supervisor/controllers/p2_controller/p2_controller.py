"""
Robótica móvil
Master en informática industrial y robótica
Universidade da Coruña
Author: Anabel Díaz Labrador
        Jaime Pablo Pérez Moro

Control del robot Khepera IV en Webots usando Supervisor.
"""

from math import radians, pi, fabs
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

    # Calcular la posición objetivo avanzando 'distance' metros en el eje Z y X
    target_position = [
        initial_position[0] + distance,
        initial_position[1],
        initial_position[2] + distance,
    ]

    # Activar los motores para mover el robot hacia adelante
    leftWheel.setVelocity(speed)
    rightWheel.setVelocity(speed)
    print("Distance: " + str(distance))

    # Bucle principal para mover el robot hacia la posición objetivo
    while robot.step(TIME_STEP) != -1:
        current_position = khepera_node.getPosition()
        print("Current position: " + str(current_position))
        if current_position[2] >= target_position[2]:
            break
        if current_position[0] >= target_position[0]:
            break
    # Detener los motores
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)
    print("Movimiento completado hacia adelante por", distance, "metros.")


def turn_left(robot, leftWheel, rightWheel, degrees, speed=CRUISE_SPEED):
    """
    Hace que el robot gire n grados a la izquierda.
    
    robot: instancia del robot.
    leftWheel, rightWheel: dispositivos de los motores de las ruedas.
    degrees: número de grados que el robot debe girar hacia la izquierda.
    speed: velocidad de las ruedas durante el giro.
    """
    # Obtener la rotación inicial del robot
    khepera_node = robot.getFromDef("Khepera")
    initial_orientation = khepera_node.getOrientation()
    
    # Calcular el ángulo objetivo en radianes
    target_angle = initial_orientation[3] - radians(degrees)  # Girar a la izquierda reduce el ángulo

    # Normalizar el ángulo objetivo para mantenerlo entre -pi y pi para facilitar las comparaciones
    if target_angle < -pi:
        target_angle += 2 * pi
    elif target_angle > pi:
        target_angle -= 2 * pi

    # Configurar las velocidades de las ruedas para el giro: la izquierda hacia atrás, la derecha hacia adelante
    leftWheel.setVelocity(-speed)
    rightWheel.setVelocity(speed)

    # Bucle principal para realizar el giro
    while robot.step(TIME_STEP) != -1:
        current_orientation = khepera_node.getOrientation()
        current_angle = current_orientation[3]

        # Normalizar el ángulo actual
        if current_angle < -pi:
            current_angle += 2 * pi
        elif current_angle > pi:
            current_angle -= 2 * pi

        # Calcular la diferencia de ángulo absoluta para verificar la cercanía al ángulo objetivo
        angle_difference = fabs(current_angle - target_angle)

        if angle_difference < radians(1):  # Permitir un margen de error en radianes
            break  # Detener el giro cuando se alcance el ángulo objetivo

    # Detener los motores
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)
    print("Giro completado a la izquierda por", degrees, "grados.")


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
