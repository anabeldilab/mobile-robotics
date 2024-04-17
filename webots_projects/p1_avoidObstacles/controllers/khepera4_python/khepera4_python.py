from controller import Robot, Motor

MAX_SPEED = 47.6

# Nombres de los sensores infrarrojos
INFRARED_SENSORS_NAMES = [
    "rear left infrared sensor", "left infrared sensor", "front left infrared sensor", "front infrared sensor",
    "front right infrared sensor", "right infrared sensor", "rear right infrared sensor", "rear infrared sensor",
    "ground left infrared sensor", "ground front left infrared sensor", "ground front right infrared sensor",
    "ground right infrared sensor"
]

# Crear una instancia del robot
robot = Robot()

# Obtener el paso de tiempo básico del mundo simulado
time_step = int(robot.getBasicTimeStep())

# Inicializar y habilitar los sensores infrarrojos
infrared_sensors = []
for sensor_name in INFRARED_SENSORS_NAMES:
    sensor = robot.getDevice(sensor_name)
    sensor.enable(time_step)
    infrared_sensors.append(sensor)

# Inicializar los motores y configurar la posición a infinito (control de velocidad)
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

while robot.step(time_step) != -1:
    # Algoritmo simple de evasión de obstáculos basado en sensores infrarrojos frontales
    speed_offset = 0.2 * (MAX_SPEED - 0.03 * infrared_sensors[3].getValue())
    speed_delta = 0.03 * infrared_sensors[2].getValue() - 0.03 * infrared_sensors[4].getValue()
    left_motor.setVelocity(speed_offset + speed_delta)
    right_motor.setVelocity(speed_offset - speed_delta)
    print("TimeStep: " + int(robot.getBasicTimeStep()))