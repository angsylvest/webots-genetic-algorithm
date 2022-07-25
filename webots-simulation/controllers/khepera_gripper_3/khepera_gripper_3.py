"""khepera_gripper controller."""

"""
Main controller Base 
Angel Sylvester 2022
"""

import random 
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor, Camera, CameraRecognitionObject, InertialUnit, GPS 
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

ds = robot.getDevice('distance sensor')
ds.enable(timestep)

# initialize emitter and reciever 
emitter = robot.getDevice("emitter")
emitter.setChannel(2)

receiver = robot.getDevice("receiver")
receiver.enable(timestep)
receiver.setChannel(1)

inertia = robot.getDevice("inertial unit")
inertia.enable(timestep)

# camera info 
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# collision info 
collision = robot.getDevice('touch sensor')
collision.enable(timestep)

# initial fitness 
global fitness
fitness = 0 

global forward_speed 
forward_speed = 2

# motor functions 

def rotate_random():
    # will choose direction following biased random walk 
    directions = [pi/2, pi, -pi/2, 0, 0, 0, 0] # more preference to move straight 
    chosen_direction = random.choice(directions)
    chosen_direction = round(chosen_direction, 2) 
    return chosen_direction 
    
def correlated_random(curr_dir): 
    # follows a markov chain (persistence) 
    # short-term straight line adherence (very simple) 
    if curr_dir == 0: 
        return random.choice([0,0, pi/2, -pi/2])
    
    elif curr_dir == round(pi/2, 2):
        return random.choice([0, pi/2, pi/2, -pi/2])
    
    elif curr_dir == round(-pi/2): 
        return random.choice([0, pi/2, -pi/2, -pi/2])
    
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
    
def stop():
    leftMotor.setPosition(float('inf'))
    leftMotor.setVelocity(0)
    rightMotor.setPosition(float('inf'))
    rightMotor.setVelocity(0)
    
# gripper functions 
def grab_object(curr_step, initial_step): 
    global fitness 
    i = curr_step - initial_step 
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
        print('fitness 3 increased', fitness) 
    elif (i == 80):
        motor.setPosition(-1.4) # arm up
        emitter.send("k3-found".encode('utf-8'))
        # delete object here 
        
    # elif (i > 100): 
        # opens the gripper 
        # leftGrip.setPosition(open_grip)
        # rightGrip.setPosition(open_grip)


def release_object(curr_step, prev_step):
    pass 
    

# Main loop:
# - perform simulation steps until Webots is stopping the controller
i = 0 
orientation_found = False 
holding_something = False
prev_i = 0 # to keep track of timesteps elapsed before new direction 
object_encountered = False 
prev_object_i = 0 # keep track of timesteps elapsed for each pickup action
chosen_direction = rotate_random()

while robot.step(timestep) != -1:

    # biased random walk movement (each time step, cert prob of turning that direction) 
    roll, pitch, yaw = inertia.getRollPitchYaw()
    yaw = round(yaw, 2) 
    
    
    if yaw != chosen_direction and orientation_found != True and object_encountered != True: 
        begin_rotating()
        
    elif (i - prev_i == 150 and object_encountered != True):
        orientation_found = False 
        chosen_direction = rotate_random()
        
    elif orientation_found != True and yaw == chosen_direction and object_encountered != True: 
        orientation_found = True 
        prev_i = i
        move_forward()
        
    else: 
        pass

    # check for collisions with other robot 
    list = camera.getRecognitionObjects()
    
    collision_status = collision.getValue()
    if collision_status == 1:
        fitness -= 1 
        
    # read distance sensor value 
    dist_val = ds.getValue()
    if dist_val < 1000 and holding_something == False: 
        stop()
        if (object_encountered == False):
            prev_object_i = i
            grab_object(i, prev_object_i)
            object_encountered = True
            
            if len(list) != 0: 
                firstObject = camera.getRecognitionObjects()[0]
           
        else: 
            grab_object(i, prev_object_i) 
            if (i - prev_object_i == 85):
                id = str(firstObject.get_id())
                print('found id')
                id = "$" + id # indication that it is a object to be deleted 
                emitter.send(str(id).encode('utf-8'))
           
                holding_something = False 
                chosen_direction = rotate_random()
                prev_object_i = i
    else: 
         object_encountered = False
         
         
    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
        
        if message[0] == "#":
            message = message[1:].split(" ")
            forward_speed = int(message[2]) 
            receiver.nextPacket()
            
        elif message == "return_fitness":
            response = "k3-fitness" + str(fitness)
            emitter.send(response.encode('utf-8'))
            receiver.nextPacket()

    
    # firstObject = camera.getRecognitionObjects()[0]
    
    # id = firstObject.get_id()
    # id = firstObject.get_model()
    # print('identified object', id)
    # position = firstObject.get_position()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

    
    i+=1
    
    pass

# Enter here exit cleanup code.