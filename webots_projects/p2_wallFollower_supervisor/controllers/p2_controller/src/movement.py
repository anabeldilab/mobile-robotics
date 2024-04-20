"""
Robótica móvil
Master en informática industrial y robótica
Universidade da Coruña
Author: Anabel Díaz Labrador
        Jaime Pablo Pérez Moro

Control del movimiento del robot Khepera IV en Webots usando Supervisor.
"""

from controller import Supervisor
from .mapping import create_map, display_map, update_map, fill_map

ORIENTATIONS = {
    "N": [[0, -1], [-1, 0]],
    "E": [[1, 0], [0, -1]],
    "S": [[0, 1], [1, 0]],
    "W": [[-1, 0], [0, 1]],
}

# Máxima velocidad de las ruedas soportada por el robot (khepera4).
MAX_SPEED = 47.6
# Velocidad por defecto para este comportamiento.
CRUISE_SPEED = 4
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


def enable_distance_sensors(robot, time_step, sensor_names):
    """
    Obtener y activar los sensores de distancia.

    Return: diccionario con los sensores de distancia activados, en el mismo orden
    establecido en la lista de nombres (sensorNames).
    """
    sensor_dict = {}

    for name in sensor_names:
        sensor_dict[name] = robot.getDevice(name)
        sensor_dict[name].enable(time_step)

    return sensor_dict


def init_devices(time_step):
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
    left_wheel = robot.getDevice("left wheel motor")
    right_wheel = robot.getDevice("right wheel motor")

    # Configuración inicial para utilizar movimiento por posición (necesario para odometría).
    # En movimiento por velocidad, establecer posición a infinito (wheel.setPosition(float('inf'))).
    left_wheel.setPosition(float("inf"))
    right_wheel.setPosition(float("inf"))
    left_wheel.setVelocity(0)
    right_wheel.setVelocity(0)

    # Obtener una lista con los sensores infrarrojos ya activados
    ir_sensor_list = enable_distance_sensors(robot, time_step, INFRARED_SENSORS_NAMES)

    # Obtener el dispositivo de la cámara
    camera = robot.getDevice("camera")
    # Activar el dispositivo de la cámara (el tiempo de actualización de los frames
    # de la cámara no debería ser muy alto debido al alto volumen de datos que implica).
    camera.enable(time_step * 10)

    # Obtener y activar los sensores de posición de las ruedas (encoders).
    pos_l = robot.getDevice("left wheel sensor")
    pos_r = robot.getDevice("right wheel sensor")
    pos_l.enable(time_step)
    pos_r.enable(time_step)

    return robot, left_wheel, right_wheel, ir_sensor_list, pos_l, pos_r, camera


def move_forward(
    robot, left_wheel, right_wheel, distance, front_ir, speed=CRUISE_SPEED
):
    """
    Mover el robot hacia adelante una distancia específica usando control supervisado.

    robot: instancia del robot.
    leftWheel, rightWheel: dispositivos de los motores de las ruedas.
    distance: distancia a moverse hacia adelante en metros.
    speed: velocidad de las ruedas.
    """
    khepera_node = robot.getFromDef("Khepera")
    # Obtener la posición inicial del robot
    initial_position = khepera_node.getPosition()

    # Activar los motores para mover el robot hacia adelante
    left_wheel.setVelocity(speed)
    right_wheel.setVelocity(speed)
    target_reached = True

    # Bucle principal para mover el robot hacia la posición objetivo
    while robot.step(TIME_STEP) != -1:
        current_position = khepera_node.getPosition()

        if current_position[2] >= initial_position[2] + distance:  # Down
            break
        if current_position[0] >= initial_position[0] + distance:  # Right
            break
        if current_position[0] <= initial_position[0] - distance:  # Left
            break
        if current_position[2] <= initial_position[2] - distance:  # Up
            break
        if front_ir.getValue() >= 250:
            # Si ha recorrido más de la mitad del camino, si se cuenta como que ha llegado
            if current_position[2] >= initial_position[2] + distance * 0.75:  # Down
                break
            if current_position[0] >= initial_position[0] + distance * 0.75:  # Right
                break
            if current_position[0] <= initial_position[0] - distance * 0.75:  # Left
                break
            if current_position[2] <= initial_position[2] - distance * 0.75:  # Up
                break
            target_reached = False
            break
    # Detener los motores
    left_wheel.setVelocity(0)
    right_wheel.setVelocity(0)
    print("Movimiento completado hacia adelante por", distance, "metros.")
    return target_reached


def change_position(current_position, orientation):
    """
    Change the position of the robot based on the current orientation.

    current_position: list with the current position of the robot.
    orientation: string with the current orientation of the robot.
    """
    if orientation == "N":
        current_position[0] -= 1
    elif orientation == "E":
        current_position[1] += 1
    elif orientation == "S":
        current_position[0] += 1
    else:
        current_position[1] -= 1
    return current_position


def get_orientation(khepera_node):
    """
    Get the orientation of the khepera_node.
    """
    orientation = khepera_node.getOrientation()
    orientation = [
        [round(orientation[0]), round(orientation[1])],
        [round(orientation[6]), round(orientation[7])],
    ]
    # Calcular la rotación objetivo
    if orientation == ORIENTATIONS["N"]:
        orientation = "N"
    elif orientation == ORIENTATIONS["E"]:
        orientation = "E"
    elif orientation == ORIENTATIONS["S"]:
        orientation = "S"
    else:
        orientation = "W"
    return orientation


def get_target_orientation(khepera_node, direction="left"):
    """
    Get the target orientation of the khepera_node.
    """
    # Obtener la rotación inicial del khepera_node
    initial_orientation = khepera_node.getOrientation()
    initial_orientation = [
        [round(initial_orientation[0]), round(initial_orientation[1])],
        [round(initial_orientation[6]), round(initial_orientation[7])],
    ]

    # Calcular la rotación objetivo
    if initial_orientation == ORIENTATIONS["N"]:
        target_orientation = (
            ORIENTATIONS["W"] if direction == "left" else ORIENTATIONS["E"]
        )
    elif initial_orientation == ORIENTATIONS["E"]:
        target_orientation = (
            ORIENTATIONS["N"] if direction == "left" else ORIENTATIONS["S"]
        )
    elif initial_orientation == ORIENTATIONS["S"]:
        target_orientation = (
            ORIENTATIONS["E"] if direction == "left" else ORIENTATIONS["W"]
        )
    else:
        target_orientation = (
            ORIENTATIONS["S"] if direction == "left" else ORIENTATIONS["N"]
        )
    return target_orientation


def turn_tolerance(current, target, tolerance):
    """
    Check if the current orientation matrix is within the specified tolerance of the target
    orientation matrix.

    :param current: 2x2 list (matrix) representing the current orientation.
    :param target: 2x2 list (matrix) representing the target orientation.
    :param tolerance: float, the tolerance for each element in the orientation matrices.
    :return: bool, True if within tolerance, False otherwise.
    """
    return all(
        abs(current[i][j] - target[i][j]) <= tolerance
        for i in range(2)
        for j in range(2)
    )


def turn(robot, left_wheel, right_wheel, speed=CRUISE_SPEED, direction="left"):
    """
    Hace que el robot gire n grados a la izquierda.

    robot: instancia del robot.
    leftWheel, rightWheel: dispositivos de los motores de las ruedas.
    speed: velocidad de las ruedas durante el giro.
    direction: dirección del giro (izquierda o derecha).
    """

    khepera_node = robot.getFromDef("Khepera")

    # Obtener la orientación target del robot
    target_orientation = get_target_orientation(khepera_node, direction)

    # Configurar las velocidades de las ruedas para el giro:
    # la izquierda hacia atrás, la derecha hacia adelante
    if direction == "left":
        left_wheel.setVelocity(-speed)
        right_wheel.setVelocity(speed)
    else:
        left_wheel.setVelocity(speed)
        right_wheel.setVelocity(-speed)

    while robot.step(TIME_STEP) != -1:
        current_orientation = khepera_node.getOrientation()
        current_orientation = [
            [round(current_orientation[0], 2), round(current_orientation[1], 2)],
            [round(current_orientation[6], 2), round(current_orientation[7], 2)],
        ]
        #print("Current orientation: ", current_orientation)
        #print("Target orientation: ", target_orientation)
        if turn_tolerance(current_orientation, target_orientation, 0.01):
            break

    # Detener las ruedas
    left_wheel.setVelocity(0)
    right_wheel.setVelocity(0)
    print("Giro completado a la ", direction)


def wall_follow(robot, left_wheel, right_wheel, ir_sensor_list, speed=CRUISE_SPEED):
    """
    Wall follow algorithm for the robot.
    Map the environment and follow the wall.

    robot: instance of the robot.
    left_wheel, right_wheel: wheel motors.
    ir_sensor_list: dictionary with the infrared sensors.
    speed: speed of the wheels.
    """

    # Map the environment with matrix
    # 0: free space
    # 1: wall
    # 2: Start
    khepera_node = robot.getFromDef("Khepera")
    env_map = create_map()
    current_position = [int(25 / 2), int(25 / 2)]
    current_orientation = get_orientation(khepera_node)

    # Check ir sensors
    while True:
        print("Current orientation: ", current_orientation)
        print("Current position: ", current_position)
        robot.step(TIME_STEP)
        front_ir = ir_sensor_list["front infrared sensor"]
        left_ir = ir_sensor_list["left infrared sensor"]
        right_ir = ir_sensor_list["right infrared sensor"]
        print("Front IR: ", front_ir.getValue())
        print("Left IR: ", left_ir.getValue())
        print("Right IR: ", right_ir.getValue())

        env_map = update_map(
            env_map, current_position, current_orientation, front_ir, left_ir, right_ir
        )

        #display_map(env_map)

        # If there is a wall in front of the robot
        if front_ir.getValue() >= 190:
            # Turn right
            turn(robot, left_wheel, right_wheel, 2, "right")
            current_orientation = get_orientation(khepera_node)
        elif left_ir.getValue() <= 160:
            # Turn left
            #print("Turning left")
            turn(robot, left_wheel, right_wheel, 2, "left")
            current_orientation = get_orientation(khepera_node)

            env_map = update_map(
                env_map, current_position, current_orientation, front_ir, left_ir, right_ir
            )

            if move_forward(
                robot, left_wheel, right_wheel, 0.25, front_ir, speed
            ):
                current_position = change_position(
                    current_position, current_orientation
                )
        else:
            if move_forward(
                robot, left_wheel, right_wheel, 0.25, front_ir, speed
            ):
                current_position = change_position(
                    current_position, current_orientation
                )

        if current_position == [int(25 / 2), int(25 / 2)]:
            print("Start position reached.")
            break

    env_map = fill_map(env_map)
    display_map(env_map)
