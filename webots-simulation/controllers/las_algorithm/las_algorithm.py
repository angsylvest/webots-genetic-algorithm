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
from framework import * 

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

ds = robot.getDevice('distance sensor') # front 
ds_right = robot.getDevice('distance sensor right')
ds_left = robot.getDevice('distance sensor left')

ds.enable(timestep)
ds_right.enable(timestep)
ds_left.enable(timestep)

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
led = robot.getDevice('led0')
led.set(1) # led to turned on 
led_1 = robot.getDevice('led1')
led_1.set(1) # led to turned on 
led_2 = robot.getDevice('led')
led_2.set(1) # led to turned on 
# led_3 = robot.getDevice('led(3)')
# led_3.set(1) # led to turned on 
# light sensor 
light_sensor = robot.getDevice('light sensor')
light_sensor.enable(timestep)
# gps info 
gps = robot.getDevice('gps')
gps.enable(timestep)

# controller parameters 
fitness = 0 
forward_speed = 5
detect_thres = 1000
time_switch = 150
obj_found_so_far = []
curr_sim_size = 5
t_block = 0
curr_sim_size = 5

# generalize id acquisition
if robot.getName() == "k0":
    given_id = 0
else: 
    given_id = robot.getName()[3:-1] 

strategy_f = open("../../graph-generation/collision-data/las-info.csv", 'a')
prev_msg = ""

# calculates angle normal to current orientation 
def calc_normal(curr_angle): 

    if (curr_angle + round(pi/2, 2) <= round(pi, 2) and curr_angle <= round(pi, 2) and curr_angle >= 0): 
        return round(curr_angle + round(pi/2, 2), 2)
    
    elif (curr_angle + round(pi/2, 2) > round(pi, 2) and curr_angle < round(pi, 2) and curr_angle > 0): 
        diff = round(pi/2, 2) - (round(pi,2) - curr_angle) 
        return round((-1*round(pi/2, 2) + diff),2)
    
    elif (curr_angle + round(pi/2, 2) < 0 and curr_angle < 0): 
        return round((-1*round(pi, 2) + curr_angle + round(pi/2, 2)),2)
        
    elif (curr_angle + round(pi/2, 2) >= 0 and curr_angle <= 0): 
        diff = abs(round(pi/2, 2) - curr_angle) 
        return round(diff,2) 
        
    elif (curr_angle == round(pi,2)): # handle edge case that seems to only happen w/exactly 3.14 (never broke before because never quite at 3.14????)
        return round(-1*round(pi/2, 2),2)
        
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
    elif (i == 80):
        motor.setPosition(-1.4) # arm up


def release_object(curr_step, prev_step):
    pass 
    
    
def interpret(): 
    global fitness
    global t_block
    global given_id
    global las
    global strategy_f
    global fitness 
    global t_block
    global curr_sim_size
    global obj_found_so_far
    global sim_complete
    global cleaning 
    global start
    
    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
    
        if message[0] == "#" + str(given_id):
            message = message[2:].split("*")
            
            las.M_vector = np.full((1, len(las.cells)), 0).tolist()
            receiver.nextPacket()
            
        elif message == "return_fitness":
            response = "k" + str(int(given_id)) + "-fitness" + str(fitness)
            emitter.send(response.encode('utf-8'))
            receiver.nextPacket()
            strategy_f.write(str(given_id) + ',' + str(robot.getTime()) + ',' + str(t_block) + ',' + str(curr_sim_size) + ',' + str(fitness)+ ',las' + ',' + str(len(obj_found_so_far)) + ',' + str(gps.getValues()[0]) + ',' + str(gps.getValues()[1]) + '\n')
            strategy_f.close()
            strategy_f = open("../../graph-generation/collision-data/las-info.csv", 'a')
            fitness = 0

        elif message == 'sim-complete':
            sim_complete = True 
            strategy_f.close()
            receiver.nextPacket()
            
        elif "trial_complete" in message:
            # resets prob distrib 
            if message[-1] == '-':
                start = False 
            obj_found_so_far = []
            receiver.nextPacket()            
            
        elif "size" in message: 
            curr_sim_size = message[5:]
            obj_found_so_far = []
            receiver.nextPacket()
            
        elif message[0] == "%" and message.split('-')[0][1:] == str(given_id):
            id = message.split('-')[1]
            obj_found_so_far.append(id)
            t_block = 0
            curr_tile = int(message.split('-')[1])
            it_passed = int(message.split('-')[2])
            holding_something = True
            
            las.reward(current_tile)
            
            if curr_tile == las.target and las.iterations_threshold <= int(it_passed):
                has_collected = True
                
            receiver.nextPacket()
            
        elif message == 'clean': 
            # want to pause controller until finished 
            cleaning = True 
            # stop()
            # print('robot has stopped, waiting for next generation')
            receiver.nextPacket()
            
        elif message == 'clean finish': 
            cleaning = False 
            # print('robot is ready to proceed') 
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

start = False
iterations_passed = 0
has_collected = False
sim_complete = False
start_count = robot.getTime()
reversing = False
moving_forward = False 
cleaning = False 

while robot.step(timestep) != -1 and sim_complete != True:
    if not cleaning: 
        interpret() 
        
        if not start and not reversing:
            las = LAS(curr_pos = (float(gps.getValues()[0]),float(gps.getValues()[1])))
            current_tile = las.locate_cell((float(gps.getValues()[0]),float(gps.getValues()[1])))
            chosen_direction = las.re_direct(current_tile)
            # print('the current tile robot is on: ', current_tile, 'target is ', las.target, 'chosen direction: ', chosen_direction) 
            start = True
        
        roll, pitch, yaw = inertia.getRollPitchYaw()
        yaw = round(yaw, 2)
        # print(yaw, 'vs: ', chosen_direction)
        current_tile = las.locate_cell((float(gps.getValues()[0]),float(gps.getValues()[1])))
        
        
        if holding_something and not reversing and not moving_forward: # move towards nest (constant vector towards home) 
            cd_x, cd_y = float(gps.getValues()[0]), float(gps.getValues()[1])
            if math.dist([cd_x, cd_y], [0,0]) > 0.05: 
                chosen_direction = round(math.atan2(-cd_y,-cd_x),2)
            else: 
                holding_something = False
                        
        if yaw != chosen_direction and object_encountered != True and orientation_found != True and not reversing: 
            begin_rotating()
            
            # handles avoidance  
        elif (i - prev_i >= 50 and object_encountered != True and orientation_found == True and not reversing and moving_forward == True):
            moving_forward = False
            orientation_found = False 
            
            # proceed with previous behavior 
            if not holding_something: 
                las.re_direct(current_tile) 
            
        elif (i - prev_i == time_switch and object_encountered != True and not reversing):
            orientation_found = False 
            
            if current_tile == las.target: 
                # chosen_direction = rotate_random()
                # orientation_found = False 
                
                if holding_something == False: 
                    iterations_passed += 1
                
                    if las.iterations_threshold <= iterations_passed:
        
                        if not has_collected: 
                            # will change direction after max number of iterations passed 
                            # iterations_passed = 0 
                            las.penalize(current_tile)
                            chosen_direction = las.re_gather(current_tile) # updates target 
                            time_switch = 150
                            
                        elif has_collected:
                            iterations_passed = 0 
                            has_collected = False # resets 
                            chosen_direction = rotate_random()
                            # orientation_found = False
                            time_switch = random.uniform(20, 50)
            else: 
                chosen_direction = las.re_direct(current_tile)  
            
        elif yaw == chosen_direction and orientation_found != True and not reversing: 
            orientation_found = True 
            prev_i = i
            move_forward()
          
        else: 
            pass
            
                
        # read distance sensor value 
        dist_val = ds.getValue()
        dist_vals = [ds.getValue(), ds_left.getValue(), ds_right.getValue()]
        
        if min(dist_vals) > 500 and reversing: # no longer within range of obstacle
            # print('proceeding with navigation')
            reversing = False
            chosen_direction = calc_normal(yaw)
            orientation_found = False 
            # begin_rotating()
            moving_forward = True 
            
        elif reversing: 
            move_backwards()
            
        if min(dist_vals) <= 330 and not reversing: # wall detection 
            fitness += 1 
            # print('collision encountered -- wall or block')
            reversing = True 
            move_backwards()   
            
            
            # does each behavior after 1 sec    
        if robot.getTime() - start_count >= 1: 
            start_count = robot.getTime()
           
            light_sensor_value = light_sensor.getValue()
            # check for collisions with other robot 
            list = camera.getRecognitionObjects()
            current_tile = las.update(current_tile, (gps.getValues()[0], gps.getValues()[1]))
         
            # handles other obstacles     
            if holding_something == False and not reversing: 
                # behavior in response to stimuli in front of robot 
                if (object_encountered == False):
                    # if retrievable object within range, gets picked up 
                    if min(dist_vals) < 500 and len(list) != 0:
                        firstObject = camera.getRecognitionObjects()[0]
                        id = str(firstObject.get_id())
                        
                        if id not in obj_found_so_far:                   
                            id = "$" + str(given_id) + "-" + str(id) + "-" + str(current_tile) + "-" + str(iterations_passed) # indication that it is a object to be deleted 
                            if id != prev_msg: 
                                emitter.send(str(id).encode('utf-8'))
                                prev_msg = id
                            
                else: 
                        t_block += 1
        
        i+=1
            
        pass
    
# Enter here exit cleanup code.
