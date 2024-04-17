from controller import Supervisor

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance

# [CODE PLACEHOLDER 1]
khepera_node = robot.getFromDef('Khepera')

while robot.step(TIME_STEP) != -1:

  position = khepera_node.getPosition()
  rotation = khepera_node.getOrientation()
  print('Khepera position: %f %f %f\n' %(position[0], position[1], position[2]))
  print('Khepera rotation 1: %f %f %f\n' %(rotation[0], rotation[1], rotation[2]))
  print('Khepera rotation 2: %f %f %f\n' %(rotation[3], rotation[4], rotation[5]))
  print('Khepera rotation 3: %f %f %f\n' %(rotation[6], rotation[7], rotation[8]))
  