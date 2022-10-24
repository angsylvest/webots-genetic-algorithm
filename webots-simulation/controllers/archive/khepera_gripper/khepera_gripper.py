"""khepera_gripper controller."""

"""
Main controller Base 
Angel Sylvester 2022
"""

import random 
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor, Camera, CameraRecognitionObject, InertialUnit 
from math import sin, cos, pi  
import random 

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

# initialize emitter and reciever 
emitter = robot.getDevice("emitter")
emitter.setChannel(2)

receiver = robot.getDevice("receiver")
receiver.enable(timestep)
receiver.setChannel(1)



i = 0 
while robot.step(timestep) != -1:
    
    i += 1
    
    # example code using gripper arm 
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
        fitness += 1 

    elif (i == 80):
        motor.setPosition(-1.4) # arm up
        
    elif (i == 100):
        leftMotor.setPosition(float('inf'))
        rightMotor.setPosition(float('inf'))
        leftMotor.setVelocity(-2)
        rightMotor.setVelocity(2)
    
    elif (i == 140):
        leftMotor.setPosition(float('inf'))
        rightMotor.setPosition(float('inf'))
        leftMotor.setVelocity(10)
        rightMotor.setVelocity(10)
    
    elif (i == 160):
        leftMotor.setPosition(float('inf'))
        rightMotor.setPosition(float('inf'))
        leftMotor.setVelocity(-2)
        rightMotor.setVelocity(2)
    
    elif (i == 200): 
        leftMotor.setPosition(float('inf'))
        rightMotor.setPosition(float('inf'))
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)    
        motor.setPosition(0) # puts arm down 
    
    elif (i == 240):
        leftGrip.setPosition(open_grip)
        rightGrip.setPosition(open_grip)
    
    elif (i == 260):
        motor.setPosition(-1.4) # puts arm down         
    
    
    pass

# Enter here exit cleanup code.
