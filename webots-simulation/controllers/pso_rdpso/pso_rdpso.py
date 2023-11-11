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
import math 

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
emitter.setChannel(4)
receiver = robot.getDevice("receiver")
receiver.enable(timestep)
receiver.setChannel(5)

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

# sim statistics 
fitness = 0 
forward_speed = 5
forward_speed_1 = 0 
forward_speed_2 = 0 
forward_speedx = 2
forward_speedy = 3

detect_thres = 1000
time_switch = 200
t_block = 0
curr_sim_size = 5

obj_found_so_far = []

prev_msg = ""

## PSO parameters ## 
individual_best = [0, 0]
group_best = [0, 0] 
group_assigned = '' # will identify group # or if in excluded group (0) 
initial_group = ''


inertial_comp = 0.2 # c1 
exploration_coefficient = 0.8 # c2 
avoidance_coefficient = 10 # c3 
magnet_connectivity_component = 0.005 # c4, r4
alpha = 0.9

avoid_pos = []
group_performance = {}
overall_population = []

# generalize id acquisition
if robot.getName() == "k0":
    given_id = 0
else: 
    given_id = robot.getName()[3:-1] 
    
emitter_individual = robot.getDevice("emitter_processor")
emitter_individual.setChannel(int(given_id)*10 + 5)
receiver_individual = robot.getDevice("receiver_processor")
receiver_individual.enable(timestep)
receiver_individual.setChannel((int(given_id) * 10) + 6)

# Agent Level File Appended Set-up 
strategy_f = open("../../graph-generation/collision-data/pso-rdpso-info.csv", 'a')

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
        print('fitness 3 increased', fitness) 
    elif (i == 80):
        motor.setPosition(-1.4) # arm up

def release_object(curr_step, prev_step):
    pass 
    
def calc_pso_params(closest_neighx, closest_neighy):
    global additive_noise 
    global forward_speedx
    global forward_speedy 
    global forward_speed_1
    global forward_speed_2
    global individual_best
    global group_best
    global alpha 
    global avoid_pos
    global avoiding 
 
    global inertial_comp  # 1
    global exploration_coefficient #2 
    global avoidance_coefficient #3  
    global magnet_connectivity_component # c4, r4 is 0 if currently in avoiding state 
    
    if not reversing: 
        coef_3 = 0
    else: 
        coef_3 = avoidance_coefficient
        
    ind_x, ind_y = individual_best
    grp_x, grp_y = group_best
    curr_posx, curr_posy = gps.getValues()[0], gps.getValues()[1]
    obs_x, obs_y = avoid_pos
    nei_x, nei_y = closest_neighx, closest_neighy

    sum_x = 0
    sum_y = 0 
     
    sum_x+= inertial_comp * random.uniform(0,1)*random.choices([-1, 1])[0] * (grp_x - curr_posx)
    sum_x+= exploration_coefficient * random.uniform(0,1)*random.choices([-1, 1])[0]  * (ind_x - curr_posx)
    sum_x+= coef_3 * random.uniform(0,1)*random.choices([-1, 1])[0] * (obs_x - curr_posx)
    sum_x+= magnet_connectivity_component * random.uniform(0,1)*random.choices([-1, 1])[0]  * (float(nei_x) - float(curr_posx))
    
    sum_y+= inertial_comp * random.uniform(0,1)*random.choices([-1, 1])[0]  * (grp_y - curr_posy)
    sum_y+= exploration_coefficient * random.uniform(0,1)*random.choices([-1, 1])[0] * (grp_y - curr_posy)
    sum_y+= coef_3 * random.uniform(0,1)*random.choices([-1, 1])[0]  * (grp_y - curr_posy)
    sum_y+= magnet_connectivity_component * random.uniform(0,1)*random.choices([-1, 1])[0]  * (float(grp_y) - float(curr_posy))
    
    forward_speed_2 = forward_speed_1 
    forward_speed_1 = forward_speed 
     
    forward_speedx = alpha*forward_speed + 0.5*alpha*forward_speed + (1/6)*alpha*forward_speed_1 * (1/24)*alpha*(1-alpha)*(2-alpha)*forward_speed_2 + sum_x  
    forward_speedy = alpha*forward_speed + 0.5*alpha*forward_speed + (1/6)*alpha*forward_speed_1 * (1/24)*alpha*(1-alpha)*(2-alpha)*forward_speed_2 + sum_y  
    
    new_x, newy = curr_posx + forward_speedx, curr_posy + forward_speedy
    
    return new_x, newy 
    
def interpret(): 
    global fitness
    global given_id 
    global t_block
    global strategy_f
    global obj_found_so_far
    global curr_sim_size
    global sim_complete
    global holding_something
    global cleaning
    global chosen_direction
    global pso_heading
    global group_assigned
    global group_dic
    global group_best 
    global receiver 
    global receiver_individual
    global emitter_individual 
    global overall_population 
    global prev_msg
    
    # sent from overall supervisor 
    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
        # print('personal msgs', message) 
            
        if message == "return_fitness":
            # print('request received') 
            response = "k" + str(given_id) + "-fitness" + str(fitness)
            # print('response is', response)
            emitter.send(response.encode('utf-8'))
            receiver.nextPacket()
            strategy_f.write(str(given_id) + ',' + str(robot.getTime()) + ',' + str(t_block) + ',' + str(curr_sim_size) + ',' + str(fitness) + ',pso-rdpso' + ',' + str(len(obj_found_so_far)) + ',' + str(gps.getValues()[0]) + ',' + str(gps.getValues()[1]) + '\n')
            strategy_f.close()
            strategy_f = open("../../graph-generation/collision-data/pso-rdpso-info.csv", 'a')
            fitness = 0
            
            receiver.nextPacket()
       
            
        elif 'group' in message: # assigns group 
            assignments = message[5:] 
            group_assigned = int(assignments[int(given_id)]) 
            initial_group = group_assigned
            obj_found_so_far = []

            # print('group assigned', group_assigned)
            emitter_individual = robot.getDevice("emitter_processor")
            emitter_individual.setChannel(int(group_assigned)*10)
            receiver_individual = robot.getDevice("receiver_processor")
            receiver_individual.enable(timestep)
            receiver_individual.setChannel((int(group_assigned) * 10) + 1)
            
            # print(emitter_individual.getChannel(), receiver_individual.getChannel())
            receiver.nextPacket()
            
        elif 'size' in message:
            curr_sim_size = message[5:]
            obj_found_so_far = []
            receiver.nextPacket()
            
                  

        elif message[0] == "%" and message.split('-')[0][1:] == str(given_id):
             
            id = message.split('-')[1]
            obj_found_so_far.append(id)
            holding_something = True
            t_block = 0
            receiver.nextPacket()
            
        elif message == 'clean': 
            # want to pause controller until finished 
            cleaning = True 
            # stop()
            # print('robot has stopped, waiting for next generation')
            receiver.nextPacket()
            
        elif message == 'trial_complete': 
            # want to reset parameters to initial values 
            # reset group number 
            # reset number collected 
            # reset relevant positions 
            obj_found_so_far = []
            group_assigned = initial_group 

            # print('group assigned', group_assigned)
            emitter_individual = robot.getDevice("emitter_processor")
            emitter_individual.setChannel(int(group_assigned)*10)
            receiver_individual = robot.getDevice("receiver_processor")
            receiver_individual.enable(timestep)
            receiver_individual.setChannel((int(group_assigned) * 10) + 1)

            receiver.nextPacket()
            
        elif 'ids' in message: 
            id_msg = message.split(" ")[1:]
            
            for id in id_msg: # will convert to nodes to eventual calculation 
                overall_population.append(int(id))
 
            receiver.nextPacket()       
            
        elif message == 'clean finish': 
            cleaning = False 
            # print('robot is ready to proceed') 
            receiver.nextPacket()
                
        else: 
            receiver.nextPacket()
           
    # sent from group supervisor 
    if receiver_individual.getQueueLength()>0:
        message = receiver_individual.getData().decode('utf-8')
        # print('group msgs to personal robot', message) 
        
        if 'pso' in message: 
            strategy_f.write(str(given_id) + ',' + str(robot.getTime()) + ',' + str(t_block) + ',' + str(curr_sim_size) + ',' + str(fitness) + ',pso-rdpso' + ',' + str(len(obj_found_so_far)) + ',' + str(gps.getValues()[0]) + ',' + str(gps.getValues()[1]) + '\n')
            strategy_f.close()
            strategy_f = open("../../graph-generation/collision-data/pso-rdpso-info.csv", 'a')
            personal_updates = message[4:].split('*')[:-1]
            
            # strip pso key word & only get location relevant to given id 
            # personal_updates = personal_updates.split('%')
            for up in personal_updates: 
                if int(given_id) == int(up.split('%')[0]): 
                    personal_updates = up.split('%') # done on purpose to catch errors if this isn't happening correctly 
            
                    psx, psy = personal_updates[1], personal_updates[2]
                    
                    # update heading to move towards direction sent 
                    curr_posx, curr_posy = gps.getValues()[0], gps.getValues()[1]
                    new_x, new_y = calc_pso_params(psx, psy)
                    
                    direction = math.atan2((new_y - curr_posx),(new_x - curr_posx))
                    chosen_direction = round(direction, 2)
                    pso_heading = round(direction, 2)
            receiver_individual.nextPacket()       
            
        elif 'punished' in message:  
            co = message[9:]
            
            ids = [int(i) for i in ''.join(co.split('*')[:-1]).split('%')[:-1]] 
            
            gp_id = message.split('*')[-1] 
            # print(ids, gp_id) 
           
            # print('punish stats', message, int(overall_population[int(given_id)]))
            if int(overall_population[int(given_id)]) in ids: # move to -1 
                # update channel so listening to new supervisor 
                # print('group assigned', group_assigned)
                group_assigned = 0 
                
                emitter_individual = robot.getDevice("emitter_processor")
                emitter_individual.setChannel(int(group_assigned)*10)
                receiver_individual = robot.getDevice("receiver_processor")
                receiver_individual.enable(timestep)
                receiver_individual.setChannel((int(group_assigned) * 10) + 1)
                
            # send back to group 
                emitter_individual.send(message.encode('utf-8'))
                    
            receiver_individual.nextPacket()
            
        elif 'rewarded' in message: # send back to find best 0 
            emitter_individual.send(message.encode('utf-8'))
            receiver_individual.nextPacket()
            
        elif 'sub-swarm' in message: # send back to find best 0 
            emitter_individual.send(message.encode('utf-8'))
            receiver_individual.nextPacket()
                
        elif 'group update' in message: 
            group_best = [message.strip('][').split(',')[0], message.strip('][').split(',')[1]]
            receiver_individual.nextPacket()
            
        elif 'subbed' in message and int(group_assigned) == 0: 
            ids = [int(i) for i in message.split('*')[1:-1]]  
            old = group_assigned
            gd = ids[given_id]
            if int(overall_population[int(given_id)]) in ids:
                group_assigned = message.split('*')[-1] 
                # print('new group assigned', group_assigned, given_id, old)
                
                # update channel so listening to new supervisor 
                emitter_individual = robot.getDevice("emitter_processor")
                emitter_individual.setChannel(int(group_assigned)*10)
                receiver_individual = robot.getDevice("receiver_processor")
                receiver_individual.enable(timestep)
                receiver_individual.setChannel((int(group_assigned) * 10) + 1)
                
                emitter_individual.send(message.encode('utf-8'))
            receiver_individual.nextPacket()
            
        else: 
            receiver_individual.nextPacket()
# Main loop:
# - perform simulation steps until Webots is stopping the controller

i = 0 
orientation_found = False 
holding_something = False
prev_i = 0 # to keep track of timesteps elapsed before new direction 
object_encountered = False 
prev_object_i = 0 # keep track of timesteps elapsed for each pickup action
chosen_direction = rotate_random()
pso_heading = 0
sim_complete = False
start_count = robot.getTime()
reversing = False 
moving_forward = False  
cleaning = False 
dis_max = 0 

while robot.step(timestep) != -1 and sim_complete != True:

    if not cleaning: 
        interpret()
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
                # print('successfully dropped off object', given_id)
        
        if yaw != chosen_direction and orientation_found != True and object_encountered != True and not reversing: 
            begin_rotating()
            
        # handles avoidance  
        elif (i - prev_i >= 50 and object_encountered != True and orientation_found == True and not reversing and moving_forward == True):
            moving_forward = False
            orientation_found = False 
            
            # proceed with previous behavior 
            if not holding_something and group_assigned != 0: 
                correlated_random(chosen_direction)
                # chosen_direction = pso_heading
            elif not holding_something: 
                 chosen_direction = pso_heading
            
    
        # exploration behavior 
        elif (i - prev_i == time_switch and object_encountered != True and not reversing and orientation_found):
            orientation_found = False
            if holding_something == False and group_assigned != 0: 
                chosen_direction = correlated_random(chosen_direction)
            elif holding_something == False: 
                chosen_direction = pso_heading
            
        elif orientation_found != True and yaw == chosen_direction and object_encountered != True and not reversing: 
            orientation_found = True 
            prev_i = i
            move_forward()
            # moving_forward = True 
        else: 
            pass
            
        # read distance sensor value 
        dist_val = ds.getValue()
        dist_vals = [ds.getValue(), ds_left.getValue(), ds_right.getValue()]
        
        if dist_val > dis_max: 
            dis_max = dist_val 
            avoid_pos = gps.getValues()[0], gps.getValues()[1]
            
        if min(dist_vals) > 500 and reversing: # no longer within range of obstacle
            # avoid_pos = gps.getValues()[0], gps.getValues()[1]
            # print('proceeding with navigation')
            reversing = False
            chosen_direction = calc_normal(yaw)
            orientation_found = False 
            moving_forward = True 
            # begin_rotating()
            
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
            
                
            # handles other obstacles     
            if holding_something == False and not reversing: 
                # behavior in response to stimuli in front of robot 
                if (object_encountered == False):
                
                    if min(dist_vals) < 500 and len(list) != 0: 
                    
                        firstObject = camera.getRecognitionObjects()[0]
                        id = str(firstObject.get_id())
                        # print('potential obj detected', id, gps.getValues()[0], gps.getValues()[1]) 
                        
                        if id not in obj_found_so_far:            
                            id = "$" + str(given_id) + "-" + str(id) + "-" + str(group_assigned) # indication that it is a object to be deleted 
                            if prev_msg != id: 
                                emitter.send(str(id).encode('utf-8'))
                                prev_msg = id 
                                # print(given_id, 'send to supervisor') 
                        
                else: 
                    t_block += 1
    
            
        i+=1
            
        pass

# Enter here exit cleanup code.
