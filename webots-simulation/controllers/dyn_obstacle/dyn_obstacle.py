from controller import Robot, Motor
from math import pi 

robot = Robot()

timestep = int(robot.getBasicTimeStep())
MAX_SPEED = 6.28

# create the Robot instance.

# get a handler to the motors and set target position to infinity (speed control)
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# set up the motor speeds at 10% of the MAX_SPEED.
leftMotor.setVelocity(0.1 * MAX_SPEED)
rightMotor.setVelocity(0.1 * MAX_SPEED)

# set up imu 
inertia = robot.getDevice("inertial unit")
inertia.enable(timestep)

n = 0 
index = 0 
start = robot.getTime() 
headings = [0, pi, -pi/2, 0, pi/2]
forward_duration = 18 

moving_forward = True 
rotating = False 

forward_speed = 5
roll, pitch, yaw = inertia.getRollPitchYaw()
yaw = round(yaw, 2)

if robot.getName() == "e-puck(1)":
    index = 2 
  

def begin_rotating():
    leftMotor.setPosition(float('inf'))
    leftMotor.setVelocity(-2)
    rightMotor.setPosition(float('inf'))
    rightMotor.setVelocity(2)
    
def move_forward():
    leftMotor.setPosition(float('inf'))
    leftMotor.setVelocity(forward_speed)
    rightMotor.setPosition(float('inf'))
    rightMotor.setVelocity(forward_speed)
    
def move_backwards(): 
    leftMotor.setPosition(float('inf'))
    leftMotor.setVelocity(-forward_speed)
    rightMotor.setPosition(float('inf'))
    rightMotor.setVelocity(-forward_speed)

    
def stop():
    leftMotor.setPosition(float('inf'))
    leftMotor.setVelocity(0)
    rightMotor.setPosition(float('inf'))
    rightMotor.setVelocity(0)
    
# runs simulation for designated amount of time 
def run_seconds(t,waiting=False):
    global n 
    global start  
    
    global rotating 
    global moving_forward 
    global index 
    
    new_t = round(t, 1)
    
    if (robot.getTime() - start < new_t): 
        move_forward() 
        moving_forward = True 
        rotating = False 
         
    else: 
        moving_forward = False 
        
        if not rotating: 
            if (index + 1) == len(headings):
                index = 0 
            else: 
                index += 1 
                
        rotating = True 
        
  
    return 
        
while robot.step(timestep) != -1:
    
    roll, pitch, yaw = inertia.getRollPitchYaw()
    yaw = round(yaw, 2)
    
    
    run_seconds(forward_duration)
    # print('yaw', yaw, 'heading', round(headings[index],2), 'index', index)
    
    if not moving_forward:   
        # now rotate 90 degrees and then return to moving forward 
        if yaw != round(headings[index],2) and rotating:
            begin_rotating() 
        else: 
            rotating = False 
            moving_forward = True 
            
            start = robot.getTime()
            

            
            
        
    
