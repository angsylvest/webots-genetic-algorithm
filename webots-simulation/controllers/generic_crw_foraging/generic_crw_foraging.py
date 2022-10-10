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

# led 
led = robot.getDevice('led')
led.set(1) # led to turned on 

# light sensor 
light_sensor = robot.getDevice('light sensor')
light_sensor.enable(timestep)

# initial fitness 
# global fitnes s
fitness = 0 
# global forward_speed 
forward_speed = 2
# global detect_thres 
detect_thres = 1000
# global time_switch
time_switch = 150
# motor functions 

global obj_found_so_far
obj_found_so_far = []

if robot.getName() == "k0":
    given_id = 0
else: 
    given_id = robot.getName()[-2] 

# Agent Level File Appended Set-up 
strategy_f = open("crw-info.csv", 'a')
t_block = 0
curr_sim_size = 5


def rotate_random():
    # will choose direction following biased random walk 
    directions = [pi/2, pi, -pi/2, 0] # more preference to move straight 
    chosen_direction = random.choice(directions)
    chosen_direction = round(chosen_direction, 2) 
    return chosen_direction 
    
def correlated_random(curr_dir): 
    # follows a markov chain (persistence) 
    # short-term straight line adherence (very simple) 
    if round(curr_dir,2) == -0.00: 
        return round(random.choice([0,0, pi/2, -pi/2]),2)
    
    elif round(curr_dir,2) == round(pi/2, 2):
        return round(random.choice([0, pi/2, pi/2, -pi/2]),2)
    
    elif round(curr_dir,2) == round(-pi/2): 
        return round(random.choice([0, pi/2, -pi/2, -pi/2]),2)
        
    else: 
        return round(random.choice([0,0, pi/2, -pi/2]),2)
    
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


def release_object(curr_step, prev_step):
    pass 
    
def parse_genotype(gen):
    global forward_speed 
    global detect_thres 
    global time_switch
    global given_id
    
    forward_speed = gen[0].count('1')
    if forward_speed < 3: 
        forward_speed = 3
    detect_thres = gen[1].count('1')
    time_switch = gen[2].count('1')
    
def interpret(): 
    global fitness
    global given_id 
    global t_block
    global strategy_f
    global obj_found_so_far
    global curr_sim_size
    
    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
    
        if message[0:2] == "#2":
            message = message[1:].split("*")
            parse_genotype(message)
            receiver.nextPacket()
            
        elif message == "return_fitness":
            print('request received') 
            response = "k" + str(given_id) + "-fitness" + str(fitness)
            print('response is', response)
            emitter.send(response.encode('utf-8'))
            receiver.nextPacket()
            strategy_f.write(str('agent id,' + str(given_id) + ',time step,' + str(robot.step(timestep)) + ',time since last block,' + str(t_block) + ',size, ' + str(curr_sim_size) + ',collisions,' + str(fitness))+ '\n')
            
            fitness = 0
            
        elif 'size' in message:
            curr_sim_size = message[4:]
            receiver.nextPacket()
            
        elif message[0] == "%" and message.split('-')[0][1:] == str(given_id):
             
            id = message.split('-')[1]
            obj_found_so_far.append(id)
            # strategy_f.write(str('agent id:' + str(given_id) + ',time step:' + str(robot.step(timestep)) + ',time since last block:' + str(t_block) + ',size: ' + str(curr_sim_size)))
                    
            # fitness = 0 
            t_block = 0
            receiver.nextPacket()
                
        else: 
            receiver.nextPacket()
    
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

    interpret()
    light_sensor_value = light_sensor.getValue()
    # biased random walk movement (each time step, cert prob of turning that direction) 
    roll, pitch, yaw = inertia.getRollPitchYaw()
    yaw = round(yaw, 2) 
    
    
    if yaw != chosen_direction and orientation_found != True and object_encountered != True: 
        begin_rotating()
        
    elif (i - prev_i == time_switch and object_encountered != True):
        orientation_found = False 
        chosen_direction = correlated_random(chosen_direction)
        
    elif orientation_found != True and yaw == chosen_direction and object_encountered != True: 
        orientation_found = True 
        prev_i = i
        move_forward()
        
    else: 
        pass

    # check for collisions with other robot 
    list = camera.getRecognitionObjects()
        
    # read distance sensor value 
    dist_val = ds.getValue()
    # print(dist_val, 'detect --', detect_thres)
    
    # wall avoidance 
    if round(dist_val) == 283:
        # fitness -= 1 
        # print('collision encountered')
        chosen_direction = rotate_random() 
        move_backwards()
        fitness += 1
        
    # handles other obstacles     
    if dist_val < detect_thres and holding_something == False and len(list) > 0: 
        # behavior in response to stimuli in front of robot 
        if (object_encountered == False):
            # prev_object_i = i
            # grab_object(i, prev_object_i)
            # object_encountered = True
            
            # if retrievable object within range, gets picked up 
            if dist_val < 40: 
                firstObject = camera.getRecognitionObjects()[0]
                # print('found object', firstObject)
                id = str(firstObject.get_id())
                
                if id not in obj_found_so_far:
                
                    # obj_found_so_far.append(id)
                    
                    # strategy_f.write(str('agent id:' + str(given_id) + ',time step:' + str(robot.step(timestep)) + ',time since last block:' + str(t_block)))
                                        
                    id = "$" + str(given_id) + "-" + str(id) # indication that it is a object to be deleted 
                    
                    emitter.send(str(id).encode('utf-8'))
                    # fitness += 1 
                    holding_something = False 
                    chosen_direction = correlated_random(chosen_direction)
                    
                    
                    # obj_found_so_far.append(id)
                    # id = "$" + id # indication that it is a object to be deleted 
                    # emitter.send(str(id).encode('utf-8'))
                    # fitness += 1 
                    # holding_something = False 
                    # chosen_direction = correlated_random(chosen_direction)
                    
            elif dist_val == 0:
                fitness += 1 
                # print('collision encountered')
                chosen_direction = rotate_random() 
                move_backwards()
                
        else: 
            t_block += 1
    else: 
         t_block += 1
         object_encountered = False
    
    i+=1
    
    pass

# Enter here exit cleanup code.
