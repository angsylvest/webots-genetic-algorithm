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
import utils.mcdt as mcdt
import utils.vector_fields as vec
import ast 

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

camera_b = robot.getDevice('camera(1)')
camera_b.enable(timestep)
camera_b.recognitionEnable(timestep)
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
repopulate = False 
# follow_thres = 3

next_child = ""
num_better = 0 
sim_type = globals.sim_type #"random" 
communication = globals.communication # False 
using_high_dens = globals.using_high_dens # True 
decentralized = globals.decentralized
num_coordination = 3 
curr_node = ""
assigned_leader = "!" 

if decentralized:
    # create tree for each type of coord 
    trees = [mcdt.DecisionTree(num_actions=n) for n in [2, 3]]
    decent_index = 0
    decent_behaviors = []
    curr_others = {}

gens_elapsed = 0 

terrains = ['normal', 'road']
current_terrain = terrains[0] # either normal or road 
prev_terrain = terrains[0]

time_as_leader = robot.getTime()
is_leader = False

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
strategy_f = open("../../graph-generation/collision-data/ga-info.csv", 'a')
gene_df = open("../../graph-generation/collision-data/ga-gene-info.csv", 'a')

# environment statistics garnered 
time_elapsed_since_block = 0 
time_elapsed_since_robot = 0
t_elapsed_constant = 10
weights = [0.25, 0.25, 0.25, 0.25] 
observations_per_strategy = [1, 1, 1, 1] # num successes using each (set to 1 so that there is still likelihood for gathering strategy) 
total_observations = sum(observations_per_strategy)
weights_high_density = [0.25, 0.25, 0.25, 0.25] 
observations_per_strategy_high_dens = [1, 1, 1, 1] # num successes using each (set to 1 so that there is still likelihood for gathering strategy) 
total_observations_high_dense = sum(observations_per_strategy_high_dens)
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
trial_num = -1 
prev_time = robot.getTime()
time_allocated = 10 # worst case 
curr_action = []
type_of_action = []
time_queued = robot.getTime()
path_length = 0
path_info = {'path_length': 0, 'num_collisions': 0}

found_something = False 

agent_observation = {'num_interactions': 0, 'num_objects_observed': 0, 'num_collisions':0}
strat_obs = {0: {"collisions": 0, "collected": 0}, 1: {"collisions": 0, "collected": 0}, 2: {"collisions": 0, "collected": 0}, 3:{"collisions": 0, "collected": 0}}
time_into_generation = 0 
using_artificial_field = False
using_bayes = globals.using_bayes
using_coordination = globals.use_coordination
remove_orientations = []
time_in_action = robot.getTime()
is_leader = False 
time_as_leader = robot.getTime()
using_vec = globals.using_vector



if using_bayes:
    multi_arm = bayes.NArmedBanditDrift(len(observations_per_strategy))

elif globals.using_ucb:
    multi_arm = mcdt.MonteCarloSingleDecisionTree(len(observations_per_strategy))

def identify_terrain(r,g,b):
    global terrains
    global current_terrain
    global prev_terrain 
    
    prev_terrain = current_terrain 
    
    if (int(r) > 170):
        # ugly hardcode to determine if on road 
        index = 0 
        current_terrain = terrains[1]
        
    else: 
        current_terrain = terrains[0]


def reward(curr_strat):
    # based around reward shaping 
    global t_elapsed_block_total
    global n_observations_block
    global fitness 
    global obj_weight 
    global obstacle_weight
    global strat_obs

    # priorities in foraging task 
    # 1. num collected using strat 
    # 2. num collisions  

    if strat_obs[curr_strat]["collisions"] != 0:
        obs_penalty = (1/strat_obs[curr_strat]["collisions"]) * 1.5
    else: 
        obs_penalty = 1.5 # reward highly lack of collisions
    goal_reward = strat_obs[curr_strat]["collected"] * 1.5

    return obs_penalty + goal_reward

def process_vect(cmd, act_type, center, leader_info=""):
    global given_id

    global decent_index
    global decent_behaviors
    global type_of_action
    global curr_action
    global forward_speed
    global is_leader

    global curr_others
    global assigned_leader
    global given_id

    global cd_x
    global cd_y

    print(f'processing cmd: {cmd} for agent {given_id}' )
    id = int(given_id)

    # for flocking, need to be designated leader
    if act_type == 0:
        if is_leader: 
            centerx, centery = center
            currx, curry = cd_x, cd_y

            ratio = 0.12
            base = 0.180
            norm = forward_speed - 5
            forward_per_sec = ratio * norm + base 
            magnitude = 1.0

            # Calculate the direction vector from the center to the particle
            direction_vector = [currx- centerx, curry - centery]
            
            # Calculate the magnitude of the direction vector
            direction_magnitude = math.sqrt(direction_vector[0]**2 + direction_vector[1]**2)
            
            # Scale the direction vector to the desired magnitude
            scaled_vector = [magnitude * direction_vector[0] / (direction_magnitude + 1e-6), magnitude * direction_vector[1] / (direction_magnitude + 1e-6)]

            angle_rad = math.atan2(scaled_vector[1], scaled_vector[0])
    
            # Ensure the angle is within [0, 2*pi) range
            if angle_rad < 0:
                angle_rad += 2 * math.pi

            dx = forward_per_sec*(time_allocated) * math.cos(angle_rad)
            dy = forward_per_sec*(time_allocated) * math.sin(angle_rad)

            goal_position = (cd_x + dx, cd_x + dy)
            curr_action = goal_position # '!'
            decent_index = -1

        else: 
            # use goal pos assigned based on output from vector field
            curr_action = cmd[id]

    elif act_type == 1: # disperse
        goal_position = cmd[id]
        decent_behaviors.append(goal_position)

        curr_action = '!'



def process_decentralized(type, node=None, action=None, neighb=None, center=None):
    global decent_index
    global decent_behaviors
    global type_of_action
    global curr_action
    global forward_speed

    global curr_others
    global assigned_leader
    global given_id

    global cd_x
    global cd_y

    goal_orientations = []
    neighbors = neighb if neighb is not None else curr_others
    length_of_action = len(action) # TODO: need to fix

    decent_behaviors = []
    decent_index = 0

    # TODO: different set up depending on if initlally called or called afterewards
    

    # set as curr_action here 
    if type == 0: # flock 
        # id type of flocking / send msg to individual supervisor to track
        if is_leader: 
            centerx, centery = center
            dir = round(math.atan2(centery-cd_y,centerx-cd_x),2) 
            filter_out = closest_reference_angle(dir)
            list_of_dir = [0.00, round(pi, 2), round(pi/2, 2), round(-pi/2, 2)]
            if filter_out in list_of_dir: # remove requesting same dir 
                list_of_dir.remove(round(filter_out,2))

            random_index = random.randint(0,2)
            act = list_of_dir[random_index]

            ratio = 0.12
            base = 0.180
            norm = forward_speed - 5
            forward_per_sec = ratio * norm + base 

            dx = forward_per_sec*(time_allocated) * math.cos(act)
            dy = forward_per_sec*(time_allocated) * math.sin(act)

            goal_position = (cd_x + dx, cd_x + dy)

            curr_action = goal_position # '!'
            decent_index = -1


        else: 
            # set goal position based on where leader is
            curr_action = neighbors[assigned_leader]
 

    # elif type == 1: # queue 
    #     # wait will be sampled 
    #     for i in range(length_of_action):
    #         decent_behaviors.append((cd_x, cd_y))
    #         # decent_behaviors[i] = (cd_x, cd_y) #  only able to stay in current pos 
    #         curr_action = '!'

    elif type == 1: # disperse 
        # distance/time based on sampled action 
        # use filtered_random function once get pos from center
        centerx, centery = center
        dir = round(math.atan2(centery-cd_y,centerx-cd_x),2) 
        filter_out = closest_reference_angle(dir)
        list_of_dir = [0.00, round(pi, 2), round(pi/2, 2), round(-pi/2, 2)]
        if filter_out in list_of_dir: # remove requesting same dir 
            list_of_dir.remove(round(filter_out,2))

        ratio = 0.12
        base = 0.180
        norm = forward_speed - 5
        forward_per_sec = ratio * norm + base 
        
        # TODO: must fix 
        for i in range(length_of_action):
            act = list_of_dir[action[i]]

            dx = forward_per_sec * math.cos(act)
            dy = forward_per_sec * math.sin(act)
        
            if i == 0: 
                goal_position = (cd_x + dx, cd_x + dy)
            # curr_action = goal_position
            else: 
                x, y = goal_position
                goal_position = (x + dx, y + dy)
            
            
            decent_behaviors.append(goal_position)
            # decent_behaviors[i] = list_of_dir[act]
        
        curr_action = '!'
        
    print(f'updated action for {given_id}: {curr_action} with type {type}')


def closest_reference_angle(dir): # center pos_x, pos_y
    angle = dir # math.atan2(pos_y - agent_y, pos_x - agent_x) 
    reference_angles = [0, round(math.pi/2,2), round(math.pi,2), round(-math.pi/2,2)]
    closest_angle = min(reference_angles, key=lambda x: abs(x - angle))

    if closest_angle == 0:
        return 0.00
    elif closest_angle == math.pi/2:
        return round(math.pi/2,2)
    elif closest_angle == math.pi:
        return round(math.pi,2)
    elif closest_angle == -math.pi/2:
        return round(-math.pi/2,2)
    else:
        return "Invalid angle"

def process_action(current_pos):
    global curr_action 
    global type_of_action 
    global time_queued
    global given_id
    global coord_status

    global is_leader
    global time_as_leader
    global cd_x
    global cd_y
    
    x,y = current_pos
    
    if type_of_action == 0: # flock
        # just continue moving to spot
        # print(f'flocking')
        
        if curr_action == 'leader':
            is_leader = True
            curr_action = []
            coord_status = True # just proceed with original movement 
        else: 
            is_leader = False 
            goalx, goaly = curr_action 
            if (math.dist([x, y], [goalx,goaly]) > 0.05): 
                coord_status = False 
            else: 
                coord_status = True
        
    elif type_of_action == 1: # queue
        # print(f'queuing')
        is_leader = False
        if robot.getTime() - time_queued >= curr_action: 
            coord_status = True 
            curr_action = [] 
            # wait
        else: 
            # can do action
            # time_queued = robot.getTime()
            coord_status = False        
    
    elif type_of_action == 2: # disperse
        # print(f'dispersing')
        is_leader = False
        goalx, goaly = curr_action 
        if (math.dist([x, y], [goalx,goaly]) > 0.05): 
            coord_status = False 
        else: 
            coord_status = True 
    
    else: 
        print('action doesnt exist')
        
    return coord_status 

def path_length_reward(path_length, alpha=0.1):
    # print(f'path length: {path_length}')
    reward = math.exp(-alpha * path_length)
    # print(f'reward: {1-neg_reward}')
    return reward


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
        
     
# parameters reset when strategy changes (after a generation) 
# fix (make fitness reliant on prior observations) 
def calc_robot_fitness():
    global t_elapsed_block_total
    global n_observations_block
    global fitness 
    global obj_weight 
    global obstacle_weight
    
    
    if fitness != 0: 
        return obj_weight*(n_observations_block) + obstacle_weight*(1 / fitness) # + 0.5*agent_observation['num_objects_observed']
    else: 
        return obj_weight*(n_observations_block) # + 0.5*agent_observation['num_objects_observed']
    
    # else: 
        # return 0 

# determines whether new strategy should be attempted based off given costs/benefits
# another idea: reward forward movement (ie., coverage? )/ time spent to complete strategy, just not as much 
def energy_expenditure():
    global energy_per_item 
    global energy_cost 
    global energy_collected_gen 
    global time_elapsed 
    
    print('avg num observed --', agent_observation['num_objects_observed'])
    if time_elapsed != 0: 
        return (energy_collected_gen*energy_per_item  - (energy_cost*time_elapsed))
    else: 
        return energy_collected_gen*energy_per_item # + (agent_observation['num_objects_observed']*energy_per_item)
    

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
        
        
def filtered_random(list_collisions): 
    # simply removes directions with obstacles while trying to re-orient 
    
    list_of_dir = [0.00, round(pi, 2), round(pi/2, 2), round(-pi/2, 2)]
    print('list of collisions --', list_collisions, list_of_dir) 
    for i in list_collisions: 
        if i in list_of_dir: # remove requesting same dir 
            list_of_dir.remove(round(i,2))
    if len(list_of_dir) == 0: 
         return round(random.choice(list_collisions), 2)   
    return round(random.choice(list_of_dir), 2)
        
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
    global observations_per_strategy 
    
    # want to update weights based off effectiveness of current strategy 
    if update: 
        if not using_bayes and not globals.using_ucb: 
            new_weights = create_new_weights(t_block, t_robot, original_weights)
            weights = new_weights 
            strat = random.choices(['straight','alternating-left','alternating-right', 'true random'], new_weights)[0]
            # print('current strat', strat)
            current_strat_index = ['straight','alternating-left','alternating-right', 'true random'].index(strat) 
            # strategy_f.write(str(given_id) + ','+ str(robot.getTime()) + ',' + str(original_weights[0]) + ',' + str(original_weights[1]) + ',' + str(original_weights[2]) + ',' + str(original_weights[3]) + ','+ str(t_block) + ',' + str(curr_sim_size) + ',' + str(calc_robot_fitness())+ ',' + str(curr_sim_size) + ',ga' +'\n')
            # strategy_f.close()
            # strategy_f = open("../../graph-generation/collision-data/ga-info.csv", 'a')
        else: # bayes weight updating 
            if using_bayes: 
                re = reward(current_strat_index)
                multi_arm.advance(current_strat_index, re) # action, reward_accum
                current_strat_index = multi_arm.sample_action()
                strat = ['straight','alternating-left','alternating-right', 'true random'][current_strat_index]
            elif globals.using_ucb:
                pass

    if not update: 
        if not using_bayes and not globals.using_ucb: 
            strat = random.choices(['straight','alternating-left','alternating-right', 'true random'], original_weights)[0]
            current_strat_index = ['straight','alternating-left','alternating-right', 'true random'].index(strat)
        else: 
            if using_bayes: 
                current_strat_index = multi_arm.sample_action()
                strat = ['straight','alternating-left','alternating-right', 'true random'][current_strat_index]
            elif globals.using_ucb:
                pass

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
        original_weights[-1] = original_weights[-1] + adjust
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
    global gene_df
    global obj_found_so_far
    global time_elapsed_since_block
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
    global curr_robot_genotype
    global n_observations_robot
    global num_better
    global repopulate
    
    global found_something
    global gens_elapsed
    global time_into_generation
    global agent_observation 

    global weights_high_density
    global observations_per_strategy_high_dens
    global sim_type
    
    global curr_index 
    global remove_orientations
    global strat_obs
    global multi_arm

    global type_of_action
    global path_length 
    global curr_action
    global time_queued
    global prev_time
    global cd_x
    global cd_y
    global forward_speed
    global time_allocated

    global is_leader
    global time_as_leader
    global holding_something
    global trees
    global curr_others
    global curr_node 
    global assigned_leader 

    global decent_behaviors
    global decent_index
    global path_info
    global using_best 

    
    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
        # print('incoming messages (controller): ', given_id, message) 
    
        # intertrial changes 
        if message[0:2] == "#" + str(given_id):
            message = message[2:].split("*")
            parse_genotype(message)
            obj_found_so_far = []
            n_observations_robot = 0
            num_better = 0 
            
            receiver.nextPacket()
            
            
        elif message == "return_fitness": # happpens at end of generation 
            if best_prev_genotype == '!': 
                best_prev_genotype = 'none'
            
            response = "k" + str(int(given_id)) + "-fitness" + str(fitness) + '-other' + str(best_prev_genotype) + '-overall' + str(calc_robot_fitness())
            # print('calculating fitness', calc_robot_fitness())
            strategy_f.write(str(given_id) + ','+ str(robot.getTime()) + ',' + str(weights[0]) + ',' + str(weights[1]) + ',' + str(weights[2]) + ',' + str(weights[3]) + ','+ str(time_elapsed_since_block) + ',' + str(n_observations_robot)  + ',' + str(curr_sim_size) + ',' + str(calc_robot_fitness())+ ',' + str(curr_sim_size) + ',ga' + ',' + str(trial_num) + ',' + str(n_observations_block) + ',' + str(curr_robot_genotype) + ',' + str(num_better) + ',' + str(gps.getValues()[0]) + ',' + str(gps.getValues()[1]) + '\n')
            strategy_f.close()
            strategy_f = open(f"../../graph-generation/collision-data/ga-info-{sim_type}-{curr_sim_size}-comm_{communication}-dense_{using_high_dens}.csv", 'a')
            
            gene_df.write(str(given_id) + ','+ str(robot.getTime()) + ',' + str(trial_num) + ',' + str(curr_sim_size) + ',' + str(curr_robot_genotype) + '\n')
            gene_df.close()
            gene_df = open(f"../../graph-generation/collision-data/ga-gene-info-{sim_type}-{curr_sim_size}-comm_{communication}-dense_{using_high_dens}.csv", 'a')

            # print('written into csvs') 
            
            fitness = 0
            overall_fitness = 0
            best_prev_genotype = '!'
            best_prev_score = -1000
            num_better = 0 
            time_into_generation = 0
            agent_observation = {'num_interactions': 0, 'num_objects_observed': 0, 'num_collisions':0}
            time_elapsed = 0 # on a per sec basis 
            
            if not found_something: 
                gens_elapsed += 1 
            
           
            if next_child != "":
                parse_genotype(next_child)
                # print('current child', next_child)
            
            emitter.send(response.encode('utf-8'))
            
            # found_something = False 
            # obj_found_so_far = []
            receiver.nextPacket()

        elif message == 'sim-complete':
            sim_complete = True 
            strategy_f.close()
            receiver.nextPacket()
            
        elif 'trial' in message: 
            # resets relevant statistics 
            trial_num = int(message[5:])
            print('end of trial, moving on to next trial', trial_num)
            strat_obs = {0: {"collisions": 0, "collected": 0}, 1: {"collisions": 0, "collected": 0}, 2: {"collisions": 0, "collected": 0}, 3:{"collisions": 0, "collected": 0}}
            gens_elapsed = 0
            fitness = 0 # number of obstacles 
            best_prev_genotype = '!'
            best_prev_score = -1000
            observations_per_strategy = [1, 1, 1, 1]
            weights = [0.25, 0.25, 0.25, 0.25] 

            weights_high_density = [0.25, 0.25, 0.25, 0.25] 
            observations_per_strategy_high_dens = [1, 1, 1, 1] # num successes using each (set to 1 so that there is still likelihood for gathering strategy) 
            
            current_strat_index = 0
            using_best = False 
            
            t_elapsed_block_total = 0
            n_observations_block = 0
            n_observations_robot = 0
            
            energy_collected_gen = 1
            num_better = 0 
            
            found_something = False 
            gens_elapsed = 0 
            
            time_elapsed = 0 # on a per sec basis 
            overall_fitness = 0
            # holding_something = False 
            obj_found_so_far = []
            curr_index = 0 
            remove_orientations = []

            decent_index = 0
            decent_behaviors = []
            curr_action = []
            curr_others = []
            path_info = {'path_length': 0, 'num_collisions': 0}
            is_leader = False

            type_of_action = 0
            msg = f"assigned-{given_id}"
            emitter_individual.send(msg.encode('utf-8'))

            if using_bayes:
                multi_arm.reset_prior()

            if globals.using_ucb:
                multi_arm.reset()
            
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
            found_something = True 
            
            # print('currently holding obj--', given_id)
            if using_high_dens and agent_observation['num_collisions'] > 0.3:
                observations_per_strategy_high_dens[current_strat_index] += 1

                incr = 0.02
                weights_high_density[current_strat_index] = weights_high_density[current_strat_index] + incr
                weights_high_density = [float(i)/sum(weights_high_density) for i in weights_high_density] 

            
            else: 
                observations_per_strategy[current_strat_index] += 1 
                obj_id = message.split('-')[1] 
                
                inc = 0.02
                weights[current_strat_index] = weights[current_strat_index] + inc
                weights = [float(i)/sum(weights) for i in weights] 

            n_observations_block += 1 
            gens_elapsed = 0
            strat_obs[current_strat_index]["collected"] += 1
            
            # observations_per_strategy[current_strat_index] += 1
            energy_collected_gen += 1
            receiver.nextPacket()
            
        elif 'generation-complete' in message: 
            msg = f"assigned-{given_id}"
            emitter_individual.send(msg.encode('utf-8'))
            receiver.nextPacket()
            
        elif 'size' in message:
            curr_sim_size = message[5:]
            obj_found_so_far = []
            trial_num = -1

            strategy_f = open(f"../../graph-generation/collision-data/ga-info-{sim_type}-{curr_sim_size}.csv", 'a')
            gene_df = open(f"../../graph-generation/collision-data/ga-gene-info-{sim_type}-{curr_sim_size}.csv", 'a')
            
            # resets relevant statistics 
            fitness = 0 # number of obstacles 
            best_prev_genotype = '!'
            best_prev_score = -1000
            observations_per_strategy = [1, 1, 1, 1]
            current_strat_index = 0
            
            t_elapsed_block_total = 0
            n_observations_block = 0 
            energy_collected_gen = 1
            n_observations_robot = 0
            num_better = 0 
            
            time_elapsed = 0 # on a per sec basis 
            overall_fitness = 0
            
            found_something = False 
            gens_elapsed = 0 
            
            receiver.nextPacket()
            

        elif 'final' in message: 
            # print(f'final msg received -- {message}')
            if (not is_leader or (robot.getTime() - time_as_leader >= time_allocated and is_leader and not decentralized) or (decentralized and curr_action == [])) and not holding_something: # curr_action == []: # if able to take on new task 
                prev_time = robot.getTime()
                
                path_length = 0 # path length reset
                path_info = {'path_length': 0, 'num_collisions': 0}
            
                dict_version = ast.literal_eval(message[12:])
                # print(f'were in actual chunk dict version: {dict_version}')
                agent_id = int(f'{given_id}')
                is_leader = False
                
            
                for key in dict_version: # it's a list for some dumb reason 
                #    print(f'key {key}')
                   cluster_dict = (dict_version[key][0])
                   strat = cluster_dict['strat_to_use']


                # print(f'strat: {strat} vs agent_id {agent_id}')
                if agent_id in strat:
                    curr_action = strat[agent_id]
                    type_of_action = cluster_dict['most_common_strat']
                    neighbors = cluster_dict['neighbors']
                    center = cluster_dict['center']
                    assignments = cluster_dict['strat_to_use']
                    vect = () # TODO: populate with curr_pose and expected dir

                    if not globals.decentralized:

                        if type_of_action == 0:
                            # assigned_leader = next((key for key, value in strat.items() if value == 'leader'), None)
                            # assigned_leader = next((key for key, value in strat.items() if value == 'leader' and key in neighbors), None)
                            # print(f'assigned leader: {assigned_leader}')
                            if curr_action == 'leader':
                                time_as_leader = robot.getTime()
                                is_leader = True

                        if type_of_action == 2: 
                            ratio = 0.12
                            base = 0.180
                            norm = forward_speed - 5
                            forward_per_sec = ratio * norm + base 
                            
                            dx = forward_per_sec * math.cos(curr_action)
                            dy = forward_per_sec * math.sin(curr_action)
                            
                            goal_position = (cd_x + dx, cd_x + dy)
                            curr_action = goal_position
                        
                        time_queued = robot.getTime()

                    else: 
                        # doing decentralized behavior with mcdt
                        if type_of_action == 0: # flock 
                            if type_of_action == 0:
                                if curr_action == 'leader':
                                    time_as_leader = robot.getTime()
                                    is_leader = True

                            assigned_leader = next(key for key, value in assignments.items() if value == 'leader')
                            leader_posx, leader_posy = neighbors[assigned_leader]
                            centerx, centery = center
                            # print(f'assigned leader: {assigned_leader}')
                            if using_vec: 
                                ratio = 0.12
                                base = 0.180
                                norm = forward_speed - 5
                                forward_per_sec = ratio * norm + base 
                                magnitude = 1.0

                                # Calculate the direction vector from the center to the particle
                                direction_vector = [leader_posx- centerx, leader_posy - centery]
                                
                                # Calculate the magnitude of the direction vector
                                direction_magnitude = math.sqrt(direction_vector[0]**2 + direction_vector[1]**2)
                                
                                # Scale the direction vector to the desired magnitude
                                scaled_vector = [magnitude * direction_vector[0] / (direction_magnitude + 1e-6), magnitude * direction_vector[1] / (direction_magnitude + 1e-6)]

                                angle_rad = math.atan2(scaled_vector[1], scaled_vector[0])
                        
                                # Ensure the angle is within [0, 2*pi) range
                                if angle_rad < 0:
                                    angle_rad += 2 * math.pi

                                dx = forward_per_sec*(time_allocated) * math.cos(angle_rad)
                                dy = forward_per_sec*(time_allocated) * math.sin(angle_rad)

                                leader_info = leader_posx, leader_posy, dx, dy

                                # update vector + goal here 
                                # print(f'inputs to vec: {neighbors} with leader {assigned_leader} with leader info {leader_info}')
                                vector_field = vec.VectorField(positions = neighbors, leader_key = assigned_leader, leader_info = leader_info )
                                attr, cmd = vector_field.calculate_attraction()
                                process_vect(cmd, type_of_action, center, leader_info)
                            else: 
                                action, node = mcdt.iterate(trees[0])
                                curr_node = node
                                process_decentralized(type_of_action, node, action, neighbors, center)

                        # elif type_of_action == 1:  # queue 
                        #     if using_best: 
                        #         action, node = mcdt.iterate_strategically(trees[0])  
                        #     else: 
                        #         action, node = mcdt.iterate(trees[0])

                        #     curr_node = node
                        #     process_decentralized(type_of_action, node, action, neighbors, center)

                        elif type_of_action == 1: # disperse 
                            if using_vec: 
                                # update vector + goal here 
                                print(f'inputs to vec: {neighbors} with leader with leader info ')
                                vector_field = vec.VectorField(positions = neighbors, leader_key = "", leader_info = "")
                                attr, cmd = vector_field.generate_dispersal(center=center)
                                process_vect(cmd, type_of_action, center)
                            else: 
                                action, node = mcdt.iterate(trees[0])
                                curr_node = node
                                process_decentralized(type_of_action, node, action, neighbors, center)
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
            
        elif 'comm_response' in message and str(message.split('-')[1]) == str(given_id) and communication: 
        
            # print('updating orientation for', given_id, 'to', message.split('[')[1], 'for given id', given_id)
            roll, pitch, yaw = inertia.getRollPitchYaw()
            yaw = round(yaw, 2)
            chosen_direction = float(message.split('[')[1])
            orientation_found = False
            receiver.nextPacket() 
      
        
        else: 
            receiver.nextPacket()
            
    if receiver_individual.getQueueLength()>0:
        message = receiver_individual.getData().decode('utf-8')
        if 'child' in message and communication: 
            next_child = message[5:].split("*")
            num_better += 1
            receiver_individual.nextPacket()
            
            
        elif 'agent' in message and 'id_ind' not in message: 
            msg = message 
            # print(f'received agent info: {msg}')
            emitter.send(str(message).encode('utf-8'))
            receiver_individual.nextPacket()
            

        elif 'neighbors_update' in message and curr_action != [] and type_of_action == 0: 
            curr_others = ast.literal_eval(message[10:])
            assigned_index = assigned_leader

            goal_posx, goal_posy = curr_others[assigned_index]

            ratio = 0.12
            base = 0.180
            norm = forward_speed - 5
            forward_per_sec = ratio * norm + base 
            
            dir = round(math.atan2(goal_posy-cd_y,goal_posx-cd_x),2) 
            dx = forward_per_sec * math.cos(dir)
            dy = forward_per_sec * math.sin(dir)

            goal_position = (cd_x + dx, cd_x + dy)
            curr_action = goal_position

            receiver_individual.nextPacket()
            
        elif 'comm' in message and str(message.split('-')[1]) == str(given_id) and not 'comm_response' in message:
            emitter.send(str(message).encode('utf-8'))
            receiver_individual.nextPacket()
            
        # if 'penalize' in message: 
            # if t_elapsed_constant < 500: 
                # t_elapsed_constant = t_elapsed_constant * 1.5 
            
            # receiver_individual.nextPacket()
            
            
        else: 
            receiver_individual.nextPacket()


def communicate_with_robot():
    global given_id 
    global prev_msg
    global chosen_direction 
    # find closest robot to exchange info with 
    response = str(given_id) + "-encounter-[" + str(chosen_direction)
    
    if prev_msg != response: 
        emitter_individual.send(response.encode('utf-8'))
        prev_msg = response 
    # print('found neighbor')
    
    
def checkForCollectable(list_of_ids):
    collectables = 0
    for obj in list_of_ids: 
        id = str(obj.get_id())
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
strategy = choose_strategy(chosen_direction, time_elapsed_since_block, time_elapsed_since_robot, weights, update = False)
curr_index = 0
initial_step_size = 500
start_count = robot.getTime()
reversing = False 
moving_forward = False  
cleaning = False
using_best = False 
prev_gen_check = robot.getTime()


cd_x, cd_y = float(gps.getValues()[0]), float(gps.getValues()[1])

while robot.step(timestep) != -1 and sim_complete != True:
    
    if not cleaning: 
        interpret(str(robot.step(timestep)))
        cd_x, cd_y = float(gps.getValues()[0]), float(gps.getValues()[1])

        if curr_action == []:
            prev_x, prev_y = cd_x, cd_y

        if curr_action != []:
            distance = math.sqrt((cd_x - prev_x)**2 + (cd_y - prev_y)**2)
            path_length += distance
            path_info['path_length'] += distance
            prev_x, prev_y = cd_x, cd_y
        
        if robot.getTime() - prev_gen_check == 1: 
            # if curr_action == []: 
                # communicate_with_robot()
            prev_gen_check = robot.getTime()
            time_into_generation += 1
            if time_into_generation % 10 == 0: 
                # time_into_generation = 0
                agent_observation = {'num_interactions': 0, 'num_objects_observed': 0, 'num_collisions':0}
            # print('given id', given_id, 'updated time into generation + here dictionary', agent_observation['num_interactions'] , 'num collisions', agent_observation['num_collisions']) 
        
        # homing mechanism 
        if holding_something == True and not reversing and not moving_forward: # move towards nest (constant vector towards home) 
            
            if math.dist([cd_x, cd_y], [0,0]) > 0.05:  
                chosen_direction = round(math.atan2(-cd_y,-cd_x),2)
                
            else: 
                holding_something = False
                t_elapsed_block_total += time_elapsed_since_block 
                # n_observations_block += 1
                time_elapsed_since_block = 0
                time_elapsed = 0 # on a per sec basis 
                # print('successfully dropped off object', given_id)
            
        if curr_index >= len(strategy) and not holding_something and not reversing and not moving_forward: # maintain strategy for initial
            curr_index = 0 
            # used to determine when to update strategy 
            print('completed strategy --', strategy, 'energy expenditure --', energy_expenditure(), 'for agent: ', given_id)
            w = weights 
            if using_high_dens and agent_observation['num_collisions'] > 0.3:
                w = weights_high_density

            if energy_expenditure() < 0: # update the weights based off success so far 
                strategy = choose_strategy(chosen_direction, time_elapsed_since_block, time_elapsed_since_robot, w, update = True) # chooses a new strategy   
                # time_elapsed = 0    
            else: 
                strategy = choose_strategy(chosen_direction, time_elapsed_since_block, time_elapsed_since_robot, w, update = False)
                # time_elapsed = 0 
    

        if not reversing and not moving_forward and not holding_something:
            
            if curr_action != [] and not decentralized: 
                done = process_action((cd_x, cd_y))
                
                if not done: 
                    if type_of_action != 1:
                        goal_posx, goal_posy = curr_action[0] + cd_x, curr_action[0] + cd_y # TODO: not correct, but logic is there 
                        
                    else: # queued behavior 
                        goal_posx, goal_posy = cd_x, cd_y
                        
                    if math.dist([cd_x, cd_y], [0,0]) > 0.05:  
                            if robot.getTime() - prev_time > time_allocated: # if unable to complete, not encouraged
                                done = True # don't add any reward since not accomplished 
                                curr_action = []
                                is_leader = False
                                msg = f'reward:{type_of_action}:{path_length_reward(path_length * 0.5)}'
                                emitter_individual.send(msg.encode('utf-8'))
                            else: 
                                # print('proceeding with original path')
                                chosen_direction = round(math.atan2(goal_posy-cd_y,goal_posx-cd_x),2) 
                    else: # request new action 
                        curr_action = []
                        is_leader = False
                        done = True
                        msg = f'reward:{type_of_action}:{path_length_reward(path_length)}'
                        emitter_individual.send(msg.encode('utf-8'))
                        print(f'finished coord task')

            if is_leader and not decentralized: 
                if robot.getTime() - time_as_leader > time_allocated: # if unable to complete, not encouraged
                    done = True # don't add any reward since not accomplished 
                    curr_action = []
                    is_leader = False
                    msg = f'reward:{type_of_action}:{path_length_reward(path_length * 0.5)}'
                    emitter_individual.send(msg.encode('utf-8'))

            if decentralized and curr_action != []: 

                if robot.getTime() - prev_time > time_allocated:
                    # continue setting chosen_direction here 
                    decent_index = 0
                    decent_behaviors = []
                    curr_action = []
                    is_leader = False

                    # update reward (based on path length )
                    
                    if path_info['num_collisions'] == 0: 
                        re = path_length_reward(path_info['path_length'])
                    else: 
                        re = path_length_reward(path_info['path_length']) * (1/path_info['num_collisions'])
                    
                    if not using_vec:
                        trees[decent_index].update_tree(curr_node, re) # TODO: make node defined

                else: 
                    if (time_allocated - robot.getTime()) % 1 == 0: 

                        # get index of decent
                        if decent_index <= (len(decent_behaviors) - 1): 

                            # TODO: update next goal 
                            if type_of_action == 0 and not is_leader: # if flocking, send msg
                                goalx, goaly = curr_action
                                if math.dist([cd_x, cd_y], [goalx, goaly]) < 0.05 and not using_vec: 
                                    # TODO: if already complete, ask for another, otherwise just continue moving towards original goal 
                                    msg = 'pos_update'
                                    emitter_individual.send(msg.encode('utf-8'))
                                    
                            elif type_of_action == 0 and is_leader: 
                                # chosen_direction = strategy[curr_index]
                                goalx, goaly = curr_action
                                chosen_direction = round(math.atan2(goaly-cd_y,goalx-cd_x),2) 
                                # curr_index += 1 
                                # curr_action = '!'
                                
                            else: 
                                goalx, goaly = decent_behaviors[decent_index]
                                chosen_direction = round(math.atan2(goaly-cd_y,goalx-cd_x),2) 
                                curr_action = '!'

                                decent_index += 1

                        elif decent_index <= (len(decent_behaviors) - 1) and math.dist([cd_x, cd_y], [goal_posx, goal_posy]) > 0.05:
                            if type_of_action == 0 and not is_leader:
                                x, y = curr_action
                                chosen_direction = round(math.atan2(y-cd_y,x-cd_x),2) 

                            elif type_of_action != 0: 
                                goalx, goaly = decent_behaviors[decent_index]
                                chosen_direction = round(math.atan2(goaly-cd_y,goalx-cd_x),2) 
                                curr_action = '!'

                        elif decent_index >= (len(decent_behaviors) - 1) and (type_of_action != 0):
                            # just continue doing what you're doing 
                            chosen_direction = strategy[curr_index]
                            curr_index += 1 
                            curr_action = '!'


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
                if curr_index >= len(strategy):
                    curr_index = 0
                    
                chosen_direction = strategy[curr_index]
                curr_action = []
                # curr_index += 1
            
        elif (i - prev_i >= time_switch and object_encountered != True and orientation_found == True and not reversing):
            orientation_found = False 
            remove_orientations = [] 
            if not holding_something and (not is_leader or curr_action == []): 
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
            if using_artificial_field and not holding_something: 
                chosen_direction = filtered_random(remove_orientations)
            else:   
                chosen_direction = calc_normal(yaw)
            orientation_found = False 
            moving_forward = True 
            
        elif reversing: 
            move_backwards()
            
        if min(dist_vals) <= 330 and not reversing: # wall detection 
            fitness += 1 
            strat_obs[current_strat_index]["collisions"] += 1
            remove_orientations.append(chosen_direction)
            reversing = True 

            path_info['num_collisions'] += 1

            move_backwards()
            if time_into_generation != 0: 
                agent_observation['num_collisions'] = (agent_observation['num_collisions'] + 1) / time_into_generation
                        
            # does each behavior after 1 sec    
        if robot.getTime() - start_count >= 1: 
            if collision.getValue() == 1:
                fitness += 1
                strat_obs[current_strat_index]["collisions"] += 1
        
            # communication threshold  
            if not holding_something and not reversing: # max value for light 
                if light_sensor.getValue() > 700 and light_sensor.getValue() < 900:
                    if time_into_generation != 0: 
                        agent_observation['num_interactions'] = (agent_observation['num_interactions'] + 1) / time_into_generation
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
                    if time_into_generation != 0: 
                        agent_observation['num_objects_observed'] = ((agent_observation['num_objects_observed']*(time_into_generation-1)) + num_not_found) / time_into_generation
                    
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

       
