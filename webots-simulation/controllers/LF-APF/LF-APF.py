"""LF-APF controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math 
from math import pi 
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
led = robot.getDevice('led')
led.set(1) # led to turned on 
# light sensor 
light_sensor = robot.getDevice('light sensor')
light_sensor.enable(timestep)
# gps info 
gps = robot.getDevice('gps')
gps.enable(timestep)

fitness = 0 
forward_speed = 5
detect_thres = 1000
time_switch = 150
obj_found_so_far = []
t_block = 0
curr_sim_size = 5

# generalize id acquisition
if robot.getName() == "k0":
    given_id = 0
else: 
    given_id = robot.getName()[3:-1] 

# Agent Level File Appended Set-up 
strategy_f = open("../../graph-generation/collision-data/levy-info.csv", 'a')
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

# variables for LF (random walk with variable step size) 
def calc_step_size():
    global forward_speed
    beta = random.uniform(0.3, 1.99)
    omega_u = ((get_gamma_val(1 + beta)*math.sin((math.pi * beta)/2)) / (beta*get_gamma_val((1 + beta)/2)*math.pow(2, ((beta - 1)/2)))**(1/beta))**2
    omega_v = 1**2
    u = sample_normal_dist(omega_u)
    v = abs(sample_normal_dist(omega_v))
    scaling_factor = 0.75**1.5
    z = abs(u / (math.pow(2,1/beta)) ) * scaling_factor
    # print('new time switch', round(((forward_speed/32)*1000) / z))
    return round(((forward_speed/32)*1000) / z)

def get_gamma_val(input): 
    return math.gamma(input) 
    
def sample_normal_dist(stdev): 
    return random.gauss(0, stdev) 
    
def rotate_random():
    # will choose direction following biased random walk 
    directions = [math.pi/2, math.pi, -math.pi/2, 0] # more preference to move straight 
    chosen_direction = random.choice(directions)
    chosen_direction = round(chosen_direction, 2) 
    return chosen_direction 
    
    
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
    global sim_complete 
    global holding_something 
    global time_switch 
    global cleaning 
    
    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
       
        # print('income robot' + str(given_id) + 'messages', message)
    
        if message[0:2] == "#2":
            message = message[1:].split("*")
            parse_genotype(message)
            receiver.nextPacket()
            
        elif message == "return_fitness":
            # print('request received') 
            response = "k" + str(given_id) + "-fitness" + str(fitness)
            # print('response is', response)
            emitter.send(response.encode('utf-8'))
            receiver.nextPacket()
            strategy_f.write(str(str(given_id) + ',' + str(robot.getTime()) + ',' + str(t_block) + ',' + str(curr_sim_size) + ',' + str(fitness) + ',levy' + ',' + str(len(obj_found_so_far)))+ ',' + str(gps.getValues()[0]) + ',' + str(gps.getValues()[1]) + '\n')
            strategy_f.close()
            strategy_f = open("../../graph-generation/collision-data/levy-info.csv", 'a')
            fitness = 0
            
        elif message == 'sim-complete':
            sim_complete = True 
            strategy_f.close()
            receiver.nextPacket()
            
        elif message == "trial_complete":
            # resets prob distrib 
            obj_found_so_far = []
            receiver.nextPacket() 
            
        elif 'size' in message:
            curr_sim_size = message[5:]
            obj_found_so_far = []
            receiver.nextPacket()
            
        elif message[0] == "%" and message.split('-')[0][1:] == str(given_id):
             
            id = message.split('-')[1]
            obj_found_so_far.append(id)
            holding_something = True   
            # print(given_id, 'holding item')      
            t_block = 0
            time_switch = 200 
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
sim_complete = False 
chosen_direction = rotate_random()
start_count = robot.getTime()
reversing = False 
moving_forward = False
cleaning = False

while robot.step(timestep) != -1 and sim_complete != True:

    if not cleaning: 
        interpret()
        light_sensor_value = light_sensor.getValue()
        # biased random walk movement (each time step, cert prob of turning that direction) 
        roll, pitch, yaw = inertia.getRollPitchYaw()
        yaw = round(yaw, 2) 
        
        if holding_something and not reversing and not moving_forward: # move towards nest (constant vector towards home) 
            cd_x, cd_y = float(gps.getValues()[0]), float(gps.getValues()[1])
            if math.dist([cd_x, cd_y], [0,0]) > 0.05: 
                chosen_direction = round(math.atan2(-cd_y,-cd_x),2)
                # print('homing --', given_id, chosen_direction, yaw)
            else: 
                holding_something = False
                # print('successfully returned', given_id) 
        
        if yaw != chosen_direction and orientation_found != True and object_encountered != True and not reversing: 
            begin_rotating()
            
            # handles avoidance  
        elif (i - prev_i >= 50 and object_encountered != True and orientation_found == True and not reversing and moving_forward == True):
            moving_forward = False
            orientation_found = False 
            
            # proceeding with previous behavior after avoidance strategy 
            if not holding_something:
                chosen_direction = rotate_random() 
                time_switch = calc_step_size()
            
        elif (i - prev_i == time_switch and object_encountered != True and not reversing):
            orientation_found = False 
            
            if not holding_something: 
                time_switch = calc_step_size()
                chosen_direction = rotate_random() 
            
        elif orientation_found != True and yaw == chosen_direction and object_encountered != True and not reversing: 
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
            # check for collisions with other robot 
    
            # handles other obstacles     
            if holding_something == False and not reversing: 
                # behavior in response to stimuli in front of robot 
                if (object_encountered == False):
                    # if retrievable object within range, gets picked up 
                    list = camera.getRecognitionObjects()
                    
                    if min(dist_vals) < 500 and len(list) != 0: 
                        firstObject = camera.getRecognitionObjects()[0]
                        # print('found object', firstObject)
                        id = str(firstObject.get_id())
                        
                        if id not in obj_found_so_far:                
                            id = "$" + str(given_id) + "-" + str(id) # indication that it is a object to be deleted 
                            if prev_msg != id: 
                                emitter.send(str(id).encode('utf-8'))
                                prev_msg = id
                            # holding_something = False 
    
                else: 
                    t_block += 1
        i+=1
        pass

# Enter here exit cleanup code.
