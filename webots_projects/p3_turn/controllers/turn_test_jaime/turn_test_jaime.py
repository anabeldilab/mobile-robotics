from controller import Supervisor
import sys
import math

def calculate_rotation_angle(current_position, target_position):
    dx = target_position[0] - current_position[0]
    dy = target_position[1] - current_position[1]
    return math.atan2(dy, dx)

def calculate_distance(current_position, target_position):
    dx = target_position[0] - current_position[0]
    dy = target_position[1] - current_position[1]
    return math.sqrt(dx**2 + dy**2)

# Inicialización del supervisor
supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep() * 2)

# Obtener el nodo del robot
robot_node = supervisor.getFromDef("Khepera")
if robot_node is None:
    sys.stderr.write("No DEF Khepera node found\n")
    sys.exit(1)

# Obteniendo campos de traducción y rotación
trans_field = robot_node.getField("translation")
rot_field = robot_node.getField("rotation")

# Motores
left_motor = supervisor.getDevice("left wheel motor")
right_motor = supervisor.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))  # Modo de velocidad
right_motor.setPosition(float('inf'))  # Modo de velocidad
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Posición y rotación inicial
INITIAL = [0, 0, 0]  # [x, y, z]
ROT = [0, 0, 1, 0]  # [x, y, z, angle]
trans_field.setSFVec3f(INITIAL)
rot_field.setSFRotation(ROT)
robot_node.resetPhysics()

# Parámetros
straight_route = [[1, 1], [1, 1.5], [2, 2]]
rotate_speed = 10.0
move_speed = 10.0
cumError_distance = 0.0

target_position = straight_route.pop(0)

# Control loop
while supervisor.step(time_step) != -1:
    current_position = trans_field.getSFVec3f()
    current_rotation = rot_field.getSFRotation()
    
    # Calcula el ángulo y la distancia al objetivo
    target_angle = calculate_rotation_angle(current_position, target_position)
    distance_error = calculate_distance(current_position, target_position)

    # CumError distance
    cumError_distance += distance_error
    
    # Diferencia de ángulo actual
    angle_difference = target_angle - current_rotation[3]
    
    # Normalizar el ángulo
    while angle_difference > math.pi:
        angle_difference -= 2 * math.pi
    while angle_difference < -math.pi:
        angle_difference += 2 * math.pi
    
    if abs(angle_difference) > 0.0001: # Rotar hacia el objetivo
        left_motor.setVelocity(-rotate_speed * angle_difference)
        right_motor.setVelocity(rotate_speed * angle_difference)
    elif distance_error > 0.01:  # Mover hacia el objetivo
        left_motor.setVelocity(move_speed * distance_error + 0.1 * cumError_distance)
        right_motor.setVelocity(move_speed * distance_error + 0.1 * cumError_distance)
    elif len(straight_route) > 0:
        print("Reached target position:", current_position)
        target_position = straight_route.pop(0)
    else:  # Detenerse cerca del objetivo
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        break

print("Reached target position:", trans_field.getSFVec3f())
