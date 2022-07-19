from controller import Supervisor, Node, Keyboard, Emitter, Receiver

"""
Main supervisor base 
Angel Sylvester 2022
"""

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance

# get info from this def 
khepera_node = robot.getFromDef("khepera")

# emitter to send info to robots 
emitter = robot.getDevice("emitter")
emitter.setChannel(1)

# set receiver to receive info 
receiver = robot.getDevice("receiver") 
receiver.enable(TIME_STEP)
receiver.setChannel(2) 

# population paramters (will need to changr) 
# POPULATION_SIZE = 15 
# GENOTYPE_SIZE = 4
# NUM_GENERATIONS = 5
# bounds = [(2,17),(-3,2),(100,180),(560,640)] 

i = 0
while robot.step(TIME_STEP) != -1:
  translation_field = khepera_node.getField('translation')
  rotation_field = khepera_node.getField('rotation')
  
  # if i == 0:
      # new_value = [2.5, 0, 0]
      # translation_field.setSFVec3f(new_value)
  

  i += 1