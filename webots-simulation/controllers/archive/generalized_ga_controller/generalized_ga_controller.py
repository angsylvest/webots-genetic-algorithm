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
import math 
import random 
# import pandas as pd 
# import numpy as np 

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

# gps info 
gps = robot.getDevice('gps')
gps.enable(timestep)

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

# initial genotype parameters

fitness = 0   
forward_speed = 2
detect_thres = 1000
time_switch = 150

sim_complete = False 
obj_found_so_far = []
curr_sim_size = 5

# given_id = robot.getName()[-1] 
if robot.getName() == "k0":
    given_id = 0
else: 
    given_id = robot.getName()[-2] 

# strategy_df = pd.DataFrame(columns = ['agent id' ,'time step', 'straight','alternating-left','alternating-right', 'true random', 'time since last block'])
# strategy_f = open(str(given_id) + "info.csv", 'w')
# strategy_f.write('agent id'+ ',time step' + ',straight' + ',alternating-left' + ',alternating-right' + ',true random' + ',time since last block' + ',size')
# strategy_f.close()

strategy_f = open("ga-info.csv", 'a')

# global time_elapsed_since_block
time_elapsed_since_block = 0
time_elapsed_since_robot = 0
weights = [0.25, 0.25, 0.25, 0.25] 
curr_best_weights = [] # set initially as empty 

def calc_step_size():
    global forward_speed
    beta = random.uniform(0.3, 1.99)
    omega_u = ((get_gamma_val(1 + beta)*math.sin((math.pi * beta)/2)) / (beta*get_gamma_val((1 + beta)/2)*math.pow(2, ((beta - 1)/2)))**(1/beta))**2
    omega_v = 1**2
    u = sample_normal_dist(omega_u)
    v = abs(sample_normal_dist(omega_v))
    
    z = abs(u / (math.pow(2,1/beta)) )
    
    return ((forward_speed/32)*1000) / z

def get_gamma_val(input): 
    return math.gamma(input) 
    
def sample_normal_dist(stdev): 
    return random.gauss(0, stdev) 

# motor functions 

def rotate_random():
    # will choose direction following biased random walk (initial) 
    directions = [pi/2, -pi/2, 0, 0] # more preference to move straight 
    chosen_direction = random.choice(directions)
    chosen_direction = round(chosen_direction, 2) 
    return chosen_direction 
    
   
def correlated_random(curr_dir): 
    # follows a markov chain (persistence), will exclude direction directly behind  
    # short-term straight line adherence (very simple) 
    if round(curr_dir,2) == -0.00 or round(curr_dir,2) == 0.00: 
        return round(random.choice([0,0, pi/2, -pi/2]),2)
    
    elif round(curr_dir,2) == round(pi/2, 2):
        return round(random.choice([pi, pi/2, pi/2, 0]),2)
    
    elif round(curr_dir,2) == round(-pi/2): 
        return round(random.choice([pi, 0, -pi/2, -pi/2]),2)
        
    else: 
        return round(random.choice([pi,pi, pi/2, -pi/2]),2)
        
def dynamic_correlated_random(curr_dir, bias): 
    # follows a markov chain (persistence), will exclude direction directly behind  
    # short-term straight line adherence (very simple)  
    
    if round(curr_dir,2) == -0.00 or round(curr_dir,2) == 0.00: 
        return round(random.choice([0, pi/2, -pi/2, bias, bias]),2)
    
    elif round(curr_dir,2) == round(pi/2, 2):
        return round(random.choice([pi, pi/2, 0, bias, bias]),2)
    
    elif round(curr_dir,2) == round(-pi/2): 
        return round(random.choice([pi, 0, -pi/2, bias, bias]),2)
        
    else: 
        return round(random.choice([pi, pi/2, -pi/2, bias, bias]),2)
        
def choose_strategy(curr_dir, t_block, t_robot, original_weights, update = False):
    global curr_best_weights
    global given_id
    global strategy_f 
    global curr_sim_size
    
    # want to update weights based off effectiveness of current strategy 
    if update: 
        new_weights = create_new_weights(t_block, t_robot, original_weights)
        strat = random.choices(['straight','alternating-left','alternating-right', 'true random'], new_weights)
        strategy_f.write('agent id:' + str(given_id) + ',time step: '+ str(robot.getTime()) + ',straight:' + str(original_weights[0]) + ',alternating-left:' + str(original_weights[1]) + ',alternating-right:' + str(original_weights[2]) + ',true random:' + str(original_weights[3]) + ',time since last block:'+ str(t_block) + ',size:' + str(curr_sim_size) + ',collisions' + str(fitness)+ ',size:' + str(curr_sim_size) +'\n')
        strategy_f.close()
        strategy_f = open("ga-info.csv", 'a')
        # new_row = {'agent id': given_id, 'time step': robot.step(timestep), 'straight': original_weights[0],'alternating-left': original_weights[1],'alternating-right': original_weights[2], 'true random': original_weights[3], 'time since last block': t_block}
        # strategy_df = pd.concat([strategy_df, pd.DataFrame([new_row])], ignore_index=True)
        
    if not update: 
        strat = random.choices(['straight','alternating-left','alternating-right', 'true random'], original_weights)
    
    if strat == 'straight':
        return [correlated_random(curr_dir)]
    elif strat == 'alternating-right':
        return [round(pi/2, 2), 0, round(-pi/2,2), round(pi,2)]
    elif strat == 'alternating-left':
        return [round(pi/2,2), round(pi,2), round(-pi/2,2), 0]
    else: # true random 
        return [random.choice([round(pi/2,2), 0, round(-pi/2,2), round(pi,2)])]
    
    
def create_new_weights(t_block, t_robot, original_weights): 
    # print('original weights --', original_weights)
    global curr_best_weights
    global weights 
    # want to incorporate some level of noise to avoid local max, instead of global
    # hope to ensure that good weights continue to persist in the pool 
     
    if len(curr_best_weights) == 0: # if there is no weight that is better (ie. just starting out) 
        new_w = []
        f = random.uniform(0, 1) 
        new_w.append(f)
        for i in range(len(original_weights)-1): 
            f = random.uniform(0, (1 - f))
            new_w.append(f)
        
        curr_best_weights = original_weights 
        curr_best_weights.append(t_block)
        curr_best_weights.append(t_robot)
        # curr_best_weights = original_weights # will serve as point of comparison for subsequent checking 
        weights = new_w
        return new_w
        
    if (curr_best_weights[-2] < t_block): # will update weights 
        curr_best_weights[:-2] = original_weights 
        curr_best_weights[-2] = t_block
        curr_best_weights[-1] = t_robot
        
        # more restrictive sampling 
        new_w = []
        f = random.uniform(original_weights[0]*0.9, original_weights[0]*1.1) 
        new_w.append(f)
        for i in range(len(original_weights)-1): 
            f = random.uniform(original_weights[i]*0.9, original_weights[i]*1.1)
            new_w.append(f)
        
        # need to normalize to 1 
        
        
        # x = np.array(new_w)
        # min_value = x.min()
        # max_value = x.max()
        
        s = sum(new_w)
        normalized = [float(i)/s for i in new_w]

        # normalized = (x - min_value) / (max_value - min_value)
        # print('normalized --', normalized)
        
        new_w = normalized
        
        weights = new_w
        return new_w
    
    else: 
        new_w = []
        f = random.uniform(0, 1) 
        new_w.append(f)
        for i in range(len(original_weights)-1): 
            f = random.uniform(0, (1 - f))
            new_w.append(f)
            
        # need to normalize to 1 
        # x = np.array(new_w)
        # min_value = x.min()
        # max_value = x.max()
        
        s = sum(new_w)
        normalized = [float(i)/s for i in new_w]

        
        new_w = normalized

        # normalized = (x - min_value) / (max_value - min_value)
        
        # new_w = normalized.tolist
        
        weights = new_w
        return new_w 
     
    
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
    
def move_back():
    leftMotor.setPosition(float('inf'))
    leftMotor.setVelocity(-forward_speed)
    rightMotor.setPosition(float('inf'))
    rightMotor.setVelocity(-forward_speed)
    
def stop():
    leftMotor.setPosition(float('inf'))
    leftMotor.setVelocity(0)
    rightMotor.setPosition(float('inf'))
    rightMotor.setVelocity(0)
    
def release_object():
    leftGrip.setPosition(open_grip)
    rightGrip.setPosition(open_grip)
    
def parse_genotype(gen):
    global forward_speed 
    global detect_thres 
    global time_switch
    
    forward_speed = gen[0].count('1')
    if forward_speed < 2: 
        forward_speed = 2
    detect_thres = gen[1].count('1')
    time_switch = gen[2].count('1')
    
def interpret(timestep): 
    global fitness
    global sim_complete
    global given_id
    global strategy_f
    global obj_found_so_far
    global time_elapsed_since_block
    global fitness
    global holding_something
    global chosen_direction
    global weights 
    global curr_sim_size
    
    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
        
        # print('incoming messages: ', given_id, message) 
    
        if message[0:2] == "#" + str(given_id):
            message = message[2:].split("*")
            parse_genotype(message)
            receiver.nextPacket()
            
        elif message == "return_fitness": # happpens at end of generation 
            response = "k" + str(int(given_id)) + "-fitness" + str(fitness)
            # print('response' , response) 
            strategy_f.write('agent id,' + str(given_id) + ',time step, '+ str(robot.getTime()) + ',straight,' + str(weights[0]) + ',alternating-left,' + str(weights[1]) + ',alternating-right,' + str(weights[2]) + ',true random,' + str(weights[3]) + ',time since last block,'+ str(time_elapsed_since_block) + ',size,' + str(curr_sim_size) + ',collisions,' + str(fitness)+ ',size,' + str(curr_sim_size) + '\n')
            strategy_f.close()
            strategy_f = open("ga-info.csv", 'a')
            
            emitter.send(response.encode('utf-8'))
            receiver.nextPacket()
            fitness = 0
            
        elif message == 'sim-complete':
            sim_complete = True 
            strategy_f.close()
            receiver.nextPacket()
            # strategy_df.to_csv('strategy_df' + str(given_id) + '.csv')
            
        elif message[0] == "%" and str(message.split('-')[0][1:]) == str(given_id):
            # strategy_f.write('agent id:' + str(given_id) + ',time step: '+ timestep + ',straight:' + str(weights[0]) + ',alternating-left:' + str(weights[1]) + ',alternating-right:' + str(weights[2]) + ',true random:' + str(weights[3]) + ',time since last block:'+ str(time_elapsed_since_block) + ',size' + str(curr_sim_size))
            
            obj_id = message.split('-')[1]
            
            obj_found_so_far.append(obj_id ) 
            time_elapsed_since_block = 0
            
            emitter.send(str(id).encode('utf-8'))
            
            # fitness += 1 
            holding_something = False 
            chosen_direction = correlated_random(chosen_direction)
            
            receiver.nextPacket()
            
        elif 'size' in message:
            curr_sim_size = message[4:]
            
            receiver.nextPacket()
        
        else: 
            receiver.nextPacket()
 
def communicate_with_robot():
    global given_id 
    # print('able to see other robot') 
    
    # find closest robot to exchange info with 
    response = str(given_id) + "-encounter"
    emitter.send(response.encode('utf-8'))
    # print('found neighbor')
    
# Main loop:
# - perform simulation steps until Webots is stopping the controller
i = 0 
orientation_found = False 
holding_something = False
prev_i = 0 # to keep track of timesteps elapsed before new direction 
object_encountered = False 
prev_object_i = 0 # keep track of timesteps elapsed for each pickup action
# global chosen_direction
chosen_direction = rotate_random()
strategy = choose_strategy(chosen_direction, time_elapsed_since_block, time_elapsed_since_robot, weights, update = False)
curr_index = 0
initial_step_size = 500
start_count = robot.getTime()

while robot.step(timestep) != -1 and sim_complete != True:
    interpret(str(robot.step(timestep))) 
    
    if curr_index >= len(strategy): 
        curr_index = 0 
        
        if robot.step(timestep) % initial_step_size == 0:
            # print('choosing strategy with update') 
            strategy = choose_strategy(chosen_direction, time_elapsed_since_block, time_elapsed_since_robot, weights, update = True) # chooses a new strategy 
            # print(strategy) 
            # initial_step_size = calc_step_size()
            
        else: 
            # print('choosing strategy, no updating') 
            strategy = choose_strategy(chosen_direction, time_elapsed_since_block, time_elapsed_since_robot, weights, update = False)
      
           # print(strategy)
    
            # interpret(str(robot.step(timestep))) # checks for messages from supervisor 
    time_elapsed_since_robot +=1
    
    # biased random walk movement (each time step, cert prob of turning that direction) 
    roll, pitch, yaw = inertia.getRollPitchYaw()
    yaw = round(yaw, 2) 

    if yaw != chosen_direction and orientation_found != True and object_encountered != True: 
        begin_rotating()
        
    elif (i - prev_i == time_switch and object_encountered != True and holding_something == False):
        orientation_found = False 
        chosen_direction = strategy[curr_index]
        curr_index += 1
    
    elif orientation_found != True and yaw == chosen_direction and object_encountered != True: 
        orientation_found = True 
        prev_i = i
        move_forward()
        
    else: 
        pass
            
            
        # does each behavior after 1 sec    
    if robot.getTime() - start_count >= 1: 
        start_count = robot.getTime()    
       
    
        # read distance sensor value 
        # want to lower fitness for those that grab other robots 
        
        # check for collisions with other robot 
        list = camera.getRecognitionObjects()
        dist_val = ds.getValue()
        # print(dist_val)
        
        if round(dist_val) == 283: # wall detection 
            fitness += 1 
            # print('collision encountered')
            chosen_direction = rotate_random() 
            move_backwards()
         
        # print('curr light values', light_sensor.getValue())   
        if time_elapsed_since_robot > 300: # max value for light 
            if light_sensor.getValue() > 800: 
                communicate_with_robot()
                time_elapsed_since_robot = 0 # reset time step 
            else: 
                time_elapsed_since_robot += 1
    
            
        if dist_val < 1000 and holding_something == False: 
            # stop()
            if (object_encountered == False):
      
                # attempt to get object detected 
                if len(list) == 1 and dist_val < 100:
                 
                    firstObject = camera.getRecognitionObjects()[0]
                    id = str(firstObject.get_id())
                    
                    if id not in obj_found_so_far:
                        # new_row = {'agent id': given_id, 'time step': robot.step(timestep), 'straight': weights[0],'alternating-left': weights[1],'alternating-right': weights[2], 'true random': weights[3], 'time since last block': time_elapsed_since_block}
                        # strategy_df = pd.concat([strategy_df, pd.DataFrame([new_row])], ignore_index=True)
                        # strategy_f.write(str('agent id': given_id, 'time step': robot.step(timestep), 'straight': weights[0],'alternating-left': weights[1],'alternating-right': weights[2], 'true random': weights[3], 'time since last block': time_elapsed_since_block))
            
                        # strategy_f.write('agent id:' + str(given_id) + ',time step: '+ str(robot.step(timestep)) + ',straight:' + str(weights[0]) + ',alternating-left:' + str(weights[1]) + ',alternating-right:' + str(weights[2]) + ',true random:' + str(weights[3]) + ',time since last block:'+ str(time_elapsed_since_block))
                        
                        # obj_found_so_far.append(id)
                        id = "$" + str(given_id) + "-" + str(id) # indication that it is a object to be deleted 
                        # time_elapsed_since_block = 0
                        
                        emitter.send(str(id).encode('utf-8'))
                        
                        # fitness += 1 
                        # holding_something = False 
                        # chosen_direction = correlated_random(chosen_direction)
             
                if dist_val == 0 or collision.getValue() == 1:
                    fitness += 1 
                    # print('collision encountered')
                    chosen_direction = rotate_random()
                    move_backwards()
            else:
                # grab_object(i, prev_object_i) 
               
                time_elapsed_since_block += 1
            
        else: 
             object_encountered = False
             time_elapsed_since_block += 1

    i+=1
    
    pass
    
# Enter here exit cleanup code.
