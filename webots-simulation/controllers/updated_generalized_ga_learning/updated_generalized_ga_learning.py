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

# ensure that we can access utils package to streamline tasks 
import sys 
sys.path.append('../../')
import utils
import utils.bayes as bayes 
import utils.globals as globals
import ast

using_bayes = globals.using_bayes

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
open_grip = 0.029
closed_grip = 0.005

# set up sensors that robot will use 
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
light_sensor = robot.getDevice('light sensor')
light_sensor.enable(timestep)

# initial genotype parameters 
forward_speed = 5
time_switch = 200

# generalize id acquisition
if robot.getName() == "k0":
    given_id = 0
    
else: 
    given_id = robot.getName()[3:-1]
    
### emitter to be sent to individual supervisor #### 
emitter_individual = robot.getDevice("emitter_processor")
emitter_individual.setChannel(int(given_id)*10)
receiver_individual = robot.getDevice("receiver_processor")
receiver_individual.enable(timestep)
receiver_individual.setChannel((int(given_id) * 10) - 1)


prev_time = robot.getTime()
time_allocated = 6 # time to move and rotate (worst case) 
iteration = 0 
fitness = 0 
curr_action = []
type_of_action = 0
time_queued = robot.getTime()
time_leading = robot.getTime()
is_leader = False

goal_posx, goal_posy = (0,0)

def determine_env_type(): # maybe eventually 
    pass

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
         
    
# direction selection 
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
    
def process_action(current_pos):
    global curr_action 
    global type_of_action 
    global time_queued
    global given_id
    # global coord_status
    global forward_speed 
    
    global goal_posy
    global goal_posx
    global is_leader
    global time_leading
    
    x,y = current_pos
    
    coord_status = False
    
    if type_of_action == 0: # flock
        # just continue moving to spot
        # print(f'flocking')
        
        if curr_action == 'leader':
            if robot.getTime() - time_leading <= 1: 
                is_leader = True
            curr_action = []
            coord_status = True # just proceed with original movement 
        else:  
            goalx, goaly = curr_action 
            if (math.dist([x, y], [goalx,goaly]) < 0.05): 
                coord_status = False 
            else: 
                coord_status = True
        
    elif type_of_action == 1: # queue
        # print(f'queuing')
        if robot.getTime() - time_queued >= curr_action: 
            coord_status = True 
            curr_action = [] 
            # wait
        else: 
            # can do action
            # time_queued = robot.getTime()
            coord_status = False        
    
    elif type_of_action == 2: # disperse
        goalx, goaly = curr_action 
        
        distance_threshold = 0.05  # Define the threshold for considering positions "very close"
        distance = math.dist([x, y], [goalx, goaly])  # Calculate the distance between current and goal positions
        if distance > distance_threshold: 
            coord_status = False  # Positions are very close, so set coord_status to False 
        else: 
            
            coord_status = True  # Positions are not very close, so set coord_status to True
            print(f'were here {distance} vs {distance_threshold} and {distance < distance_threshold}')
        # if (math.dist([x, y], [goalx,goaly]) < 0.05): 
            # coord_status = False 
        # else: 
            # coord_status = True 
            
        print(f'coord status: {coord_status} given current pos {[x, y]} and goal pos {[goalx,goaly]} with dist {math.dist([x, y], [goalx,goaly])}')
    
    else: 
        print('action doesnt exist')
        
    return coord_status 

def path_length_reward(path_length, alpha=0.1):
    neg_reward = math.exp(-alpha * path_length)
    return 1 - neg_reward

def interpret(timestep): 
    global fitness
    global given_id
    global obj_found_so_far
    global holding_something
    global curr_sim_size
    
    global overall_fitness
    
    global n_observations_block
    global cleaning 
    global trial_num 
    global curr_robot_genotype
    global n_observations_robot
    global repopulate
    
    global found_something
    global agent_observation 
    global sim_type

    global curr_action 
    global goal_posx
    global goal_posy
    global type_of_action
    global time_queued
    
    global forward_per_sec
    global prev_act
    global iteration
    global path_length
    global forward_speed
    global cd_x
    global cd_y
    global time_allocated
    
    global time_leading
    global is_leader
    
    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
        # print('incoming messages: ', given_id, message) 
        
        # want to enforce strategy by adding for bias to it here     
        if message[0] == "%" and str(message.split('-')[0][1:]) == str(given_id):
            # strategy_f.write('agent id:' + str(given_id) + ',time step: '+ timestep + ',straight:' + str(weights[0]) + ',alternating-left:' + str(weights[1]) + ',alternating-right:' + str(weights[2]) + ',true random:' + str(weights[3]) + ',time since last block:'+ str(time_elapsed_since_block) + ',size' + str(curr_sim_size))
            holding_something = True 
            found_something = True 
            n_observations_block += 1 
            
            receiver.nextPacket()
            
        elif 'size' in message:
            curr_sim_size = message[5:]
            obj_found_so_far = []
            trial_num = -1
            
            n_observations_block = 0 
            n_observations_robot = 0
            overall_fitness = 0
            found_something = False 
            
            receiver.nextPacket()
            
        elif 'final' in message: 
            if True: # curr_action == []: # if able to take on new task 
                msg = f'reward:{type_of_action}:{path_length_reward(path_length)}'
                emitter_individual.send(msg.encode('utf-8'))
                
                path_length = 0 # path length reset
            
                dict_version = ast.literal_eval(message[12:])
                agent_id = int(f'{given_id}')
                
                curr_strategy_proposed = {}
                
                orientation = curr_action 
                time_allocated = robot.getTime()
                
            
                for key in dict_version: 
                #    print(f'key {key}')
                   cluster_dict = (dict_version[key][0])
                   strat = cluster_dict['strat_to_use']
                   if agent_id in strat:
                       curr_action = (strat[agent_id])
                       type_of_action = cluster_dict['most_common_strat']
                       
                       if time_of_action == 0: 
                           if curr_action == 'leader': 
                               time_leading = robot.getTime()
                               is_leader = True
                       
                       if type_of_action == 2: 
                            ratio = 0.12
                            base = 0.180
                            norm = forward_speed - 5
                            forward_per_sec = (ratio*norm + base)*6
                            
                            print(f'curr orientation: {curr_action}')
                            
                            dx = forward_per_sec * math.cos(curr_action)
                            dy = forward_per_sec * math.sin(curr_action)
                            
                            goal_position = (cd_x + dx, cd_x + dy)
                            curr_action = goal_position
                            
                            print(f'dispersing to goal {curr_action} from {cd_x,cd_y}')
                           
                       time_queued = robot.getTime()
                
                # print(f'dict version {dict_version}: {agent_id} agent id {agent_id} with next_action: {curr_action} for strat {type_of_action}')

            else: 
                print('ignoring, other action still in progress')
            receiver.nextPacket()
        
            
        elif message == 'clean': 
            # want to pause controller until finished 
            cleaning = True 
            receiver.nextPacket()
            
        elif message == 'clean finish': 
            cleaning = False 
            receiver.nextPacket() 
        
        else: 
            receiver.nextPacket()
            
    if receiver_individual.getQueueLength()>0:
        message = receiver_individual.getData().decode('utf-8')
        # message = receiver_individual.getString()

        if 'episode-agent-complete' in message:
            # reset agent info 
            curr_action = [] # set to nothing initially
            type_of_action = 0
            
            receiver_individual.nextPacket()
            # TODO: fill out with valid reset to agent actions

        elif 'final-delib' in message: 
            # process input to determine next action 
            # print(f'final delib info: {message}')
            receiver_individual.nextPacket() 
        # coordination task 
        # need schedule and task (if anything)
            
        else: 
            receiver_individual.nextPacket()

def communicate_with_robot():
    global given_id 
    global prev_msg
    global chosen_direction 
    global curr_action 
    
    global time_leading
    global is_leader
    
    complete = False
    
    if (curr_action == [] and not is_leader) or (is_leader and (robot.getTime() - time_leading >= 6):
        is_leader = False 
        complete = True
        
    # find closest robot to exchange info with 
    response = str(given_id) + "-encounter-[" + str(chosen_direction)
    if prev_msg != response: 
        emitter_individual.send(response.encode('utf-8'))
        prev_msg = response 
    # print('found neighbor')
        
def checkForCollectable(list_of_ids):
    collectables = 0
    for obj in list_of_ids: 
        id = str(obj.getId())
        if id not in obj_found_so_far:
            collectables += 1
    return collectables 

# controller tracker variables 
i = 0 
prev_i = 0 # to keep track of timesteps elapsed before new direction 
back_i = 0
orientation_found = False 
holding_something = False
object_encountered = False 
prev_object_i = 0 # keep track of timesteps elapsed for each pickup action
# global chosen_direction
chosen_direction = rotate_random()
# strategy = choose_strategy(chosen_direction, time_elapsed_since_block, time_elapsed_since_robot, weights, update = False)
curr_index = 0
initial_step_size = 500
start_count = robot.getTime()
reversing = False 
moving_forward = False  
cleaning = False
prev_gen_check = robot.getTime() 
prev_act = ""

time_into_exploration = 0 
just_begun = True 
# coord_status = False
t_elapsed_constant = 10
time_elapsed_since_robot = 0
time_elapsed = 0 
prev_msg = ""
path_length = 0 
cd_x = 0
cd_y = 0


while robot.step(timestep) != -1:
    
    if not cleaning: 
        interpret(str(robot.step(timestep)))
        
        cd_x, cd_y = float(gps.getValues()[0]), float(gps.getValues()[1])
        if just_begun: # TODO: make more aligned with status of env
            # send individual supervisor agent id index 
            msg = f"assigned-{given_id}"
            emitter_individual.send(msg.encode('utf-8'))
            just_begun = False
            prev_x, prev_y = cd_x, cd_y
        
        if curr_action != []:
            distance = math.sqrt((cd_x - prev_x)**2 + (cd_y - prev_y)**2)
            path_length += distance
            prev_x, prev_y = cd_x, cd_y
            
        if robot.getTime() - prev_gen_check == 10: 
            communicate_with_robot() # temporarily added here for debugging 
            prev_gen_check = robot.getTime()
            time_into_exploration += 1

        # homing mechanism 
        if holding_something == True and not reversing and not moving_forward and curr_action == []: # move towards nest (constant vector towards home) 
            cd_x, cd_y = float(gps.getValues()[0]), float(gps.getValues()[1])
            if math.dist([cd_x, cd_y], [0,0]) > 0.05:  
                chosen_direction = round(math.atan2(-cd_y,-cd_x),2)
                
            else: 
                holding_something = False

        # do action sequence 
        if not holding_something and not reversing and not moving_forward and curr_action == []: 
            # print('proceeding with original task') 
            # goal_posx, goal_posy = curr_action[0] + cd_x, curr_action[0] + cd_y # TODO: not correct, but logic is there 
            if math.dist([cd_x, cd_y], [0,0]) > 0.05:  
                if robot.getTime() - prev_time > time_allocated: # if unable to complete, not encouraged
                    pass 
                else: 
                    # print('proceeding with original path')
                    chosen_direction = round(math.atan2(0-cd_y,0-cd_x),2) 
            else: # request new action 
                pass
           
        roll, pitch, yaw = inertia.getRollPitchYaw()
        yaw = round(yaw, 2)  
           
        if not holding_something and not reversing and not moving_forward and curr_action != []:
            done = process_action((cd_x, cd_y))
            # print(f'done {done} vs chosen_direction {chosen_direction} vs {yaw}')
            
            if not done: 
                if type_of_action != 1:
                    #goal_posx, goal_posy = curr_action[0] + cd_x, curr_action[0] + cd_y # TODO: not correct, but logic is there 
                    pass 
                    
                else: 
                    goal_posx, goal_posy = cd_x, cd_y
                    
                if math.dist([cd_x, cd_y], [goal_posx,goal_posy]) > 0.05 and type_of_action != 1:  
                        # if robot.getTime() - prev_time > time_allocated: # if unable to complete, not encouraged
                            # print(f'time elapsed')
                            # curr_action = []
                            # done = True
                             
                        # else: 
                        chosen_direction = round(math.atan2(round(goal_posy-cd_y,2)-cd_y,goal_posx-cd_x),2) 
                        print(f'proceeding with original path with chosen direction {chosen_direction}')
                else: # request new action 
                    curr_action = []
                    done = True
                    print(f'finished coord task')
            
           
    
        if yaw != chosen_direction and orientation_found != True and object_encountered != True and not reversing: 
            begin_rotating()
            
        # handles avoidance  
        elif (i - back_i >= 50 and object_encountered != True and orientation_found == True and not reversing and moving_forward):
            moving_forward = False
            # proceeds with previous strategy 
            orientation_found = False 
            if not holding_something: 
                chosen_direction = round(math.atan2(-cd_y,-cd_x),2) 
            
        elif (i - prev_i >= time_switch and object_encountered != True and orientation_found == True and not reversing):
            orientation_found = False 
            remove_orientations = [] 
            if not holding_something: 
                chosen_direction = round(math.atan2(-cd_y,-cd_x),2) 
        
        elif orientation_found != True and yaw == chosen_direction and object_encountered != True and not reversing: 
            orientation_found = True 
            prev_i = i
            # for avoidance strategy 
            back_i = i 
            move_forward()    
            # moving_forward = True 
            
        # collision avoidance mechanism     
        dist_val = ds.getValue()
        dist_vals = [ds.getValue(), ds_left.getValue(), ds_right.getValue()]
               
        # TODO: need to remove collision avoidance, should be part of learning as well? 

        if min(dist_vals) > 500 and reversing: # no longer within range of obstacle
            # print('proceeding with navigation')
            reversing = False
            # if using_artificial_field and not holding_something: 
            #     chosen_direction = filtered_random(remove_orientations)
            # else:   

            chosen_direction = calc_normal(yaw)
            orientation_found = False 
            moving_forward = True 
            
        elif reversing: 
            move_backwards()
            
        if min(dist_vals) <= 330 and not reversing: # wall detection 
            fitness += 1 
            reversing = True 
            move_backwards()
                        
   
        if robot.getTime() - start_count >= 1: 
            # if collision.getValue() == 1:
                # fitness += 1
                # strat_obs[current_strat_index]["collisions"] += 1
        
            # communication threshold  
            if not holding_something and not reversing: # max value for light 
                if light_sensor.getValue() > 700 and light_sensor.getValue() < 900:
                    # if time_into_generation != 0: 
                        # agent_observation['num_interactions'] = (agent_observation['num_interactions'] + 1) / time_into_generation
                    if time_elapsed_since_robot > t_elapsed_constant: 
                        communicate_with_robot()
                        time_elapsed_since_robot = 0 # reset time step      
                # elif light_sensor.getValue() > 800: 
                time_elapsed_since_robot += 1 # increment every time (more interactions) 
                
                
            start_count = robot.getTime()  
            if not holding_something:  # don't take into account homing state 
                time_elapsed += 1
            
            # check for collisions with other robot 
            list = camera.getRecognitionObjects()
            if holding_something == False and not reversing: 
                # stop()
                if (object_encountered == False):
                    # frequency of observing foragable objects 
                    num_not_found = checkForCollectable(camera.getRecognitionObjects())
                    # if time_into_generation != 0: 
                        # agent_observation['num_objects_observed'] = ((agent_observation['num_objects_observed']*(time_into_generation-1)) + num_not_found) / time_into_generation
                    
                    # attempt to get object detected 
                    if min(dist_vals) < 500 and len(list) != 0:
                        firstObject = camera.getRecognitionObjects()[0]
                        count = len(camera.getRecognitionObjects())
                        id = str(firstObject.get_id())
                        if id not in obj_found_so_far:
                            id = "$" + str(given_id) + "-" + str(id) # indication that it is a object to be deleted 
                            if prev_msg != id: 
                                emitter.send(str(id).encode('utf-8'))
                                prev_msg = id 
                        # else: 
                            # time_elapsed_since_block += 1 # on a per sec basis    
                    # else: 
                        # time_elapsed_since_block += 1 # on a per sec basis 
        i+=1
            
        pass

    
# Enter here exit cleanup code.



"""

import random 
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor, Camera, CameraRecognitionObject, InertialUnit 
from math import sin, cos, pi 
import math 
import random 

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
open_grip = 0.029
closed_grip = 0.005

# set up sensors that robot will use 
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
light_sensor = robot.getDevice('light sensor')
light_sensor.enable(timestep)

# initial genotype parameters 
forward_speed = 5
# detect_thres = 1000
time_switch = 200

sim_complete = False 
obj_found_so_far = []
curr_sim_size = 5
potential_time = 90
# follow_thres = 3

next_child = ""

# generalize id acquisition
if robot.getName() == "k0":
    given_id = 0
    
else: 
    given_id = robot.getName()[3:-1]
    
### emitter to be sent to individual supervisor #### 
emitter_individual = robot.getDevice("emitter_processor")
emitter_individual.setChannel(int(given_id)*10)
receiver_individual = robot.getDevice("receiver_processor")
receiver_individual.enable(timestep)
receiver_individual.setChannel((int(given_id) * 10) - 1)

# collects statistics about strategy and collisions 
strategy_f = open("../../graph-generation/collision-data/ga-info-learning.csv", 'a')
gene_df = open("../../graph-generation/collision-data/ga-gene-learner-info.csv", 'a')

# environment statistics garnered 
time_elapsed_since_block = 0 
time_elapsed_since_robot = 0
weights = [0.25, 0.25, 0.25, 0.25] 
observations_per_strategy = [1, 1, 1, 1] # num successes using each (set to 1 so that there is still likelihood for gathering strategy) 
total_observations = sum(observations_per_strategy)
current_strat_index = 0 # set arbitrarily 
curr_best_weights = [] # set initially as empty 

# statistics to be updated each second 
t_elapsed_block_total = 0 # gathered over sec
n_observations_block = 0 # will be used to calulate avg over a sec   
n_observations_robot = 0 # will be used to calulate avg over a sec 
t_elapsed_robot_total = 0 # gathered over sec 

best_prev_genotype = '!'
best_prev_score = -1000 
curr_robot_genotype = []

# fitness statistics 
obj_weight = 2
obstacle_weight = 1 
num_observations = 0 # will be used to
fitness = 0  # represents total # of collisions so far 
observations_threshold = 5
overall_fitness = 0

# MVT intuition statistics (overall) 
energy_per_item = 10 
energy_cost = 0.5 # per sec 
energy_collected_gen = 0
time_elapsed = 0 # on a per sec basis

# prev message (limit overloading receiver/emitter system) 
prev_msg = "" 
trial_num = 0 

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
        
     
# parameters reset when strategy changes (after a generation) 
def calc_robot_fitness():
    global t_elapsed_block_total
    global n_observations_block
    global fitness 
    global obj_weight 
    global obstacle_weight
    
    if fitness != 0: 
        return obj_weight*(n_observations_block) + obstacle_weight*(1 / fitness)
    else: 
        return obj_weight*(n_observations_block) 
    
    # else: 
        # return 0 

# determines whether new strategy should be attempted based off given costs/benefits
def energy_expenditure():
    global energy_per_item 
    global energy_cost 
    global energy_collected_gen 
    global time_elapsed 
    
    if time_elapsed != 0: 
        return (energy_collected_gen*energy_per_item - (energy_cost*time_elapsed))
    else: 
        return energy_collected_gen*energy_per_item
    

# direction selection 
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
   
# strategy selection      
def choose_strategy(curr_dir, t_block, t_robot, original_weights, update = False):
    global curr_best_weights
    global given_id
    global strategy_f 
    global curr_sim_size
    global current_strat_index 
    global weights 
    
    # want to update weights based off effectiveness of current strategy 
    if update: 
        new_weights = create_new_weights(t_block, t_robot, original_weights)
        weights = new_weights 
        strat = random.choices(['straight','alternating-left','alternating-right', 'true random'], new_weights)[0]
        print('current strat', strat)
        current_strat_index = ['straight','alternating-left','alternating-right', 'true random'].index(strat) 
        # strategy_f.write(str(given_id) + ','+ str(robot.getTime()) + ',' + str(original_weights[0]) + ',' + str(original_weights[1]) + ',' + str(original_weights[2]) + ',' + str(original_weights[3]) + ','+ str(t_block) + ',' + str(curr_sim_size) + ',' + str(calc_robot_fitness())+ ',' + str(curr_sim_size) + ',ga' +'\n')
        # strategy_f.close()
        # strategy_f = open("../../graph-generation/collision-data/ga-info.csv", 'a')

    if not update: 
        strat = random.choices(['straight','alternating-left','alternating-right', 'true random'], original_weights)[0]
        current_strat_index = ['straight','alternating-left','alternating-right', 'true random'].index(strat)
    
    if strat == 'straight':
        return [curr_dir, curr_dir, curr_dir, curr_dir]
    elif strat == 'alternating-right':
        return [round(pi/2, 2), 0, round(-pi/2,2), round(pi,2)]
    elif strat == 'alternating-left':
        return [round(pi/2,2), round(pi,2), round(-pi/2,2), 0]
    else: #  correlated random
        return [correlated_random(curr_dir), correlated_random(curr_dir), correlated_random(curr_dir), correlated_random(curr_dir)]
    
    
def create_new_weights(t_block, t_robot, original_weights): 
    # print('original weights --', original_weights)
    global curr_best_weights
    global weights 
    global observations_per_strategy
    global total_observations 
    global current_strat_index
    global observations_per_strategy
    # want to incorporate some level of noise to avoid local max, instead of global
    # hope to ensure that good weights continue to persist in the pool 
    
    if sum(observations_per_strategy) >= observations_threshold: 
        new_w = [observations_per_strategy[i] + 0.25 for i in range(len(observations_per_strategy))]
        new_w = [observations_per_strategy[i]/sum(observations_per_strategy) for i in range(len(observations_per_strategy))]
        return new_w
    
    # want to set weights so more biased towards straight-line motion (no energy)  
    else: 
        adjust = 0.02
        original_weights[-1] = original_weights[-1] + 0.02 
        return [float(i)/sum(original_weights) for i in original_weights] 
         
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
    global energy_per_item 
    global energy_cost 
    global observations_threshold 
    global observations_threshold
    global curr_robot_genotype
    
    curr_robot_genotype = gen
    
    forward_speed = gen[0].count('1')
    if forward_speed < 5: 
        forward_speed = 5
    energy_cost = gen[1].count('1')
    energy_per_item = gen[2].count('1')
    observations_threshold = gen[3].count('1')
    
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
    global strategy 
    global weights 
    global curr_sim_size
    global best_prev_genotype
    global best_prev_score  
    
    global observations_per_strategy
    global total_observations 
    global current_strat_index 
    global overall_fitness
    
    global energy_collected_gen
    global time_elapsed
    global t_elapsed_block_total
    global n_observations_block
    global cleaning 
    global next_child
    global trial_num 
    global potential_time
    
    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
        # print('incoming messages: ', given_id, message) 
    
        # intertrial changes 
        if message[0:2] == "#" + str(given_id):
            message = message[2:].split("*")
            parse_genotype(message)
            
            receiver.nextPacket()
            
        elif message == "return_fitness": # happpens at end of generation 
            if best_prev_genotype == '!': 
                best_prev_genotype = 'none'
            
            response = "k" + str(int(given_id)) + "-fitness" + str(fitness) + '-other' + str(best_prev_genotype) + '-overall' + str(calc_robot_fitness())
            print('calculating fitness', calc_robot_fitness())
            strategy_f.write(str(given_id) + ','+ str(robot.getTime()) + ',' + str(weights[0]) + ',' + str(weights[1]) + ',' + str(weights[2]) + ',' + str(weights[3]) + ','+ str(time_elapsed_since_block) + ',' + str(curr_sim_size) + ',' + str(calc_robot_fitness())+ ',' + str(curr_sim_size) + ',ga' + ',' + str(trial_num) + ',' + str(n_observations_block) + ',' + str(potential_time) + '\n')
            strategy_f.close()
            strategy_f = open("../../graph-generation/collision-data/ga-info-learning.csv", 'a')
            
            gene_df.write(str(given_id) + ','+ str(robot.getTime()) + ',' + str(trial_num) + ',' + str(curr_sim_size) + ',' + str(curr_robot_genotype) + '\n')
            gene_df.close()
            gene_df = open("../../graph-generation/collision-data/ga-gene-info.csv", 'a')

            fitness = 0
            overall_fitness = 0
            best_prev_genotype = '!'
            best_prev_score = -1000
            
           
            if next_child != "":
                parse_genotype(next_child)
                print('current child', next_child)
            
            emitter.send(response.encode('utf-8'))
            
            obj_found_so_far = []
            receiver.nextPacket()

        elif message == 'sim-complete':
            sim_complete = True 
            strategy_f.close()
            receiver.nextPacket()
            
        elif 'trial' in message: 
            # resets relevant statistics 
            trial_num = int(message.split('-')[0][5:])
            potential_time = int(message.split('-')[1])
            fitness = 0 # number of obstacles 
            best_prev_genotype = '!'
            best_prev_score = -1000
            observations_per_strategy = [1, 1, 1, 1]
            current_strat_index = 0
            
            t_elapsed_block_total = 0
            n_observations_block = 0
            
            energy_collected_gen = 1
            
            time_elapsed = 0 # on a per sec basis 
            overall_fitness = 0
            obj_found_so_far = []
            receiver.nextPacket()
            
        elif 'partner' in message: 
            # assesses if this is best so far in generation 
            msg = message.split('-')
            other_robot_index = msg[1]
            other_fitness = msg[2]
            other_collected_count = msg[3]
            
            if (other_collected_count - other_fitness) > best_prev_score: 
                best_prev_genotype = other_robot_index 
                best_prev_score = other_collected_count
            receiver.nextPacket()
        
        # want to enforce strategy by adding for bias to it here     
        elif message[0] == "%" and str(message.split('-')[0][1:]) == str(given_id):
            # strategy_f.write('agent id:' + str(given_id) + ',time step: '+ timestep + ',straight:' + str(weights[0]) + ',alternating-left:' + str(weights[1]) + ',alternating-right:' + str(weights[2]) + ',true random:' + str(weights[3]) + ',time since last block:'+ str(time_elapsed_since_block) + ',size' + str(curr_sim_size))
            holding_something = True 
            print('currently holding obj--', given_id)
            observations_per_strategy[current_strat_index] += 1
            
            obj_id = message.split('-')[1] 
            obj_found_so_far.append(obj_id) 
            inc = 0.02
            weights[current_strat_index] = weights[current_strat_index] + inc
            weights = [float(i)/sum(weights) for i in weights] 
            
            # observations_per_strategy[current_strat_index] += 1
            energy_collected_gen += 1
            receiver.nextPacket()
            
        elif 'size' in message:
            curr_sim_size = message[4:]
            obj_found_so_far = []
            
            # resets relevant statistics 
            fitness = 0 # number of obstacles 
            best_prev_genotype = '!'
            best_prev_score = -1000
            observations_per_strategy = [1, 1, 1, 1]
            current_strat_index = 0
            
            t_elapsed_block_total = 0
            n_observations_block = 0 
            energy_collected_gen = 1
            
            time_elapsed = 0 # on a per sec basis 
            overall_fitness = 0
            
            receiver.nextPacket()
            
        elif message == 'clean': 
            # want to pause controller until finished 
            cleaning = True 
            receiver.nextPacket()
            
        elif message == 'clean finish': 
            cleaning = False 
            receiver.nextPacket() 
        
        else: 
            receiver.nextPacket()
            
    if receiver_individual.getQueueLength()>0:
        message = receiver_individual.getData().decode('utf-8')
        if 'child' in message: 
            next_child = message[5:].split("*")
            receiver_individual.nextPacket()
        else: 
            receiver_individual.nextPacket()
            
            

def communicate_with_robot():
    global given_id 
    global prev_msg
    # find closest robot to exchange info with 
    response = str(given_id) + "-encounter"
    if prev_msg != response: 
        emitter_individual.send(response.encode('utf-8'))
        prev_msg = response 
    # print('found neighbor')

# controller tracker variables 
i = 0 
prev_i = 0 # to keep track of timesteps elapsed before new direction 
back_i = 0
orientation_found = False 
holding_something = False
object_encountered = False 
prev_object_i = 0 # keep track of timesteps elapsed for each pickup action
# global chosen_direction
chosen_direction = rotate_random()
strategy = choose_strategy(chosen_direction, time_elapsed_since_block, time_elapsed_since_robot, weights, update = False)
curr_index = 0
initial_step_size = 500
start_count = robot.getTime()
reversing = False 
moving_forward = False  
cleaning = False

while robot.step(timestep) != -1 and sim_complete != True:

    if not cleaning: 
        interpret(str(robot.step(timestep)))
        
        # homing mechanism 
        if holding_something == True and not reversing and not moving_forward: # move towards nest (constant vector towards home) 
            cd_x, cd_y = float(gps.getValues()[0]), float(gps.getValues()[1])
            if math.dist([cd_x, cd_y], [0,0]) > 0.05:  
                chosen_direction = round(math.atan2(-cd_y,-cd_x),2)
                
            else: 
                holding_something = False
                t_elapsed_block_total += time_elapsed_since_block 
                n_observations_block += 1
                time_elapsed_since_block = 0
                time_elapsed = 0 # on a per sec basis 
                print('successfully dropped off object', given_id)
            
        
        if curr_index >= len(strategy) and not holding_something and not reversing and not moving_forward: # maintain strategy for initial
            curr_index = 0 
            # used to determine when to update strategy 
            if energy_expenditure() < 0: # update the weights based off success so far 
                strategy = choose_strategy(chosen_direction, time_elapsed_since_block, time_elapsed_since_robot, weights, update = True) # chooses a new strategy   
                # time_elapsed = 0    
            else: 
                strategy = choose_strategy(chosen_direction, time_elapsed_since_block, time_elapsed_since_robot, weights, update = False)
                # time_elapsed = 0 
    
        time_elapsed_since_robot +=1
        # biased random walk movement (each time step, cert prob of turning that direction) 
        roll, pitch, yaw = inertia.getRollPitchYaw()
        yaw = round(yaw, 2) 
    
        if yaw != chosen_direction and orientation_found != True and object_encountered != True and not reversing: 
            begin_rotating()
            
            # handles avoidance  
        elif (i - back_i >= 50 and object_encountered != True and orientation_found == True and not reversing and moving_forward):
            moving_forward = False
            # proceeds with previous strategy 
            orientation_found = False 
            if not holding_something: 
                chosen_direction = strategy[curr_index]
            
        elif (i - prev_i == time_switch and object_encountered != True and orientation_found == True and not reversing):
            orientation_found = False 
            if not holding_something: 
                chosen_direction = strategy[curr_index]
                curr_index += 1
        
        elif orientation_found != True and yaw == chosen_direction and object_encountered != True and not reversing: 
            orientation_found = True 
            prev_i = i
            # for avoidance strategy 
            back_i = i 
            move_forward()    
            # moving_forward = True 
            
        # collision avoidance mechanism     
        dist_val = ds.getValue()
        dist_vals = [ds.getValue(), ds_left.getValue(), ds_right.getValue()]
                
        if min(dist_vals) > 500 and reversing: # no longer within range of obstacle
            # print('proceeding with navigation')
            reversing = False
            chosen_direction = calc_normal(yaw)
            orientation_found = False 
            moving_forward = True 
            
        elif reversing: 
            move_backwards()
            
        if min(dist_vals) <= 330 and not reversing: # wall detection 
            fitness += 1 
            reversing = True 
            move_backwards()
                        
            # does each behavior after 1 sec    
        if robot.getTime() - start_count >= 1: 
            # communication threshold  
            if not holding_something and not reversing: # max value for light 
                if light_sensor.getValue() > 700 and light_sensor.getValue() < 900:
                    if time_elapsed_since_robot > 500: 
                        communicate_with_robot()
                        time_elapsed_since_robot = 0 # reset time step      
                elif light_sensor.getValue() > 800: 
                    time_elapsed_since_robot += 1 
                
                
            start_count = robot.getTime()  
            if not holding_something:  # don't take into account homing state 
                time_elapsed += 1
            
            # check for collisions with other robot 
            list = camera.getRecognitionObjects()
            if holding_something == False and not reversing: 
                # stop()
                if (object_encountered == False):
                    # attempt to get object detected 
                    if min(dist_vals) < 500 and len(list) != 0:
                        firstObject = camera.getRecognitionObjects()[0]
                        count = len(camera.getRecognitionObjects())
                        id = str(firstObject.get_id())
                        if id not in obj_found_so_far:
                            id = "$" + str(given_id) + "-" + str(id) # indication that it is a object to be deleted 
                            if prev_msg != id: 
                                emitter.send(str(id).encode('utf-8'))
                                prev_msg = id 
                        else: 
                            time_elapsed_since_block += 1 # on a per sec basis    
                    else: 
                        time_elapsed_since_block += 1 # on a per sec basis 
        i+=1
            
        pass
    
# Enter here exit cleanup code.
"""

