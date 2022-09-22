"""dynamic_env_gen controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor
import random

# create the Robot instance.
robot = Supervisor()
block_list = []

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

def generate_robot_central(num_robots):
    for i in range(num_robots):
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../supervisor_controller/robots/robot-ga.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        t_field.setSFVec3f([round(random.uniform(0.5, -0.5),2), round(random.uniform(0.5, -0.5) ,2), 0.2])
        
    pass 
    
def regenerate_blocks_random():

    global block_list
    for obj in block_list: 
        obj.remove()
    
    block_list = []
    
    # floor_size = arena_area.getField('floorSize')
    # print('arena size --', floor_size.getSFVec2f()) 
    # tile_size = arena_area.getField('floorTileSize')
    # print('tile size --', tile_size.getSFVec2f()) 
    
    # generates block on opposite sides of arena (randomly generated) 
    for i in range(10): 
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../supervisor_controller/cylinder-obj.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        t_field.setSFVec3f([round(random.uniform(0.9, -0.9),2), round(random.uniform(0.3, 0.85),2), 0.02]) 
        block_list.append(rec_node)
    
    for i in range(10): 
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../supervisor_controller/cylinder-obj.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        t_field.setSFVec3f([round(random.uniform(0.9, -0.9),2), round(random.uniform(-1, 0.23),2), 0.02]) 
        block_list.append(rec_node)
        
    
def generate_blocks_single_source():
    pass
    
def generate_blocks_dual_source():
    pass 

# Main loop:
# - perform simulation steps until Webots is stopping the controller

i = 0
while robot.step(timestep) != -1:
    if (i == 0): # test to generate robots and blocks 
        # regenerate_blocks_random()
        generate_robot_central(1)
        
    i += 1 
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
