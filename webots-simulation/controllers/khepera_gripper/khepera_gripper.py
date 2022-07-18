"""khepera_gripper controller."""

"""
Main Code Base 
Angel Sylvester 2022
"""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor, Camera, CameraRecognitionObject 

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
open_grip = 0.029
closed_grip = 0.005

motor = robot.getDevice('motor')

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

leftGrip = robot.getDevice('left grip')
rightGrip = robot.getDevice('right grip')

ds = robot.getDevice('ds')
ds.enable(timestep)


# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
i = 0 

while robot.step(timestep) != -1:
    if (i == 0):
        # opens the gripper 
        leftGrip.setPosition(open_grip)
        rightGrip.setPosition(open_grip)
        
    elif (i == 20):
        motor.setPosition(0) # arm down 
    
    elif (i == 40):
        # closes the gripper 
        leftGrip.setPosition(closed_grip)
        rightGrip.setPosition(closed_grip) 
    
    elif (i == 80):
        motor.setPosition(-1.4) # arm up
        
    elif (i == 100):
        leftMotor.setPosition(float('inf'))
        leftMotor.setVelocity(-2) # turns robot while backing
        rightMotor.setPosition(float('inf'))
        rightMotor.setVelocity(2)
        
    elif (i == 140):
        leftMotor.setPosition(float('inf'))
        leftMotor.setVelocity(10) # moves robot forward
        rightMotor.setPosition(float('inf'))
        rightMotor.setVelocity(10)
 
        
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    
    # read distance sensor value 
    dist_val = ds.getValue()
    
    # camera info 
    camera = robot.getDevice('camera')
    camera.enable(timestep)
    camera.recognitionEnable(timestep)
    firstObject = camera.getRecognitionObjects()
    print(firstObject)
    # id = firstObject.get_id()
    # position = firstObject.get_position()
    
    
    # detect obstacle 
    
    # detect other robot 
    

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    
    i+=1
    
    pass

# Enter here exit cleanup code.
