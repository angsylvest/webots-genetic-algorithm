from controller import Supervisor, Node, Keyboard, Emitter, Receiver, Field
# import statistics 
# import pandas as pd
import random
import math 

"""
Main supervisor base 
Optimization algorithm - Collaboration-oriented 
Angel Sylvester 2022
"""

# collected counts csv generation 
overall_f = open('../../graph-generation/collection-data/overall-ant-info.csv', 'w') 
overall_f.write('trial' + ',time' + ',objects retrieved' + ',size' + ',type' + '\n')
overall_f.close()
overall_f = open('../../graph-generation/collection-data/overall-ant-info.csv', 'a') 

# individual robot collision counts  
strategy_f = open("../../graph-generation/collision-data/ant-info.csv", 'w')
strategy_f.write('agent id,'+ 'time step,' +' time since last block' + ',size' + ',collisions'+ ',type' + ',collected' + ',pos x' + ',pos y' + '\n')
strategy_f.close()

# set-up robot
TIME_STEP = 32
robot = Supervisor()  # create Supervisor instance
# emitter to send info to robots 
emitter = robot.getDevice("emitter")
emitter.setChannel(1)
# set receiver to receive info 
receiver = robot.getDevice("receiver") 
receiver.enable(TIME_STEP)
receiver.setChannel(2) 
arena_area = robot.getFromDef("arena")

# set up timing so consistent with ga 
num_generations = 20
simulation_time = 30
trials = 100
curr_size = 5
robot_population_sizes = [10, 15] # [5, 10, 15]
curr_trial = 0 

# sim statistics 
total_collected = 0 
taken = False # getting second child assigned 
total_found = 0
updated = False
fit_update = False 
count = 0
found_list = []
block_list = []
collected_count = []
r_pos_to_generate = []
b_pos_to_generate = []
b_pos_to_generate_alternative = []
population = []
fitness_scores = []
start = 0
prev_msg = "" 
random.seed(11) # was 11 (changed to 15 to get new configuration) 
assessing = True # change once finished gathering data  
repopulate = False 

def generate_robot_central(num_robots):
    global fitness_scores 
    global collected_count 
    global population
    global overall_columns 
    global r_pos_to_generate
    
    # initialize_genotypes(num_robots)
    emitter.send(str("size-" + str(num_robots)).encode('utf-8'))
    if len(population) != 0: 
    
        for r in population: 
            r.remove()

    population = []
    fitness_scores = []
    collected_count = []
    
    for i in range(num_robots):
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../las_supervisor/robots/robot-ant.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        pos = [round(random.uniform(0.25, -0.25),2), round(random.uniform(0.25, -0.25) ,2), 0.02]
        r_pos_to_generate.append(pos)
        t_field.setSFVec3f(pos)
        
        # sets up metrics 
        fitness_scores.append("!")
        collected_count.append(0)
        population.append(rec_node)
        

def regenerate_environment(block_dist): # will stay constant based off seed 
    # creates a equally distributed set of blocks 
    # avoiding areas where a robot is already present 
    global block_list
    global population 
    global r_pos_to_generate
    global b_pos_to_generate
    
    for obj in block_list: 
        obj.remove()
    
    block_list = []
    
    for i in range(len(r_pos_to_generate)):
        population[i].getField('translation').setSFVec3f(r_pos_to_generate[i])
        
        
    # generates block on opposite sides of arena (randomly generated) 
    if len(b_pos_to_generate) == 0: 
        for i in range(10): 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            pose = [round(random.uniform(0.9, -0.9),2), round(random.uniform(0.3, 0.85),2), 0.02]
            t_field.setSFVec3f(pose) 
            b_pos_to_generate.append(pose)
            block_list.append(rec_node)
        
        for i in range(10): 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            pose = [round(random.uniform(0.9, -0.9),2), round(random.uniform(-1, 0.23),2), 0.02]
            t_field.setSFVec3f(pose) 
            b_pos_to_generate.append(pose)
            block_list.append(rec_node)
            
    
    else: 
        # if already generated, use the previously saved positions 
        for i in b_pos_to_generate: 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            t_field.setSFVec3f(i) 
            block_list.append(rec_node)
     
# initially reads from file for re-producibility, and then continues to re-read (will be second)        
def regenerate_environment_alternate(block_dist): # will stay constant based off seed 
    # creates a equally distributed set of blocks 
    # avoiding areas where a robot is already present 
    global block_list
    global population 
    global r_pos_to_generate
    global b_pos_to_generate_alternative
    
    for obj in block_list: 
        obj.remove()
    
    block_list = []
    
    for i in range(len(r_pos_to_generate)):
        population[i].getField('translation').setSFVec3f(r_pos_to_generate[i])
        
    # generates block on opposite sides of arena (randomly generated) 
    if len(b_pos_to_generate_alternative) == 0: 
        seed_file = open('../../graph-generation/seed-15-pl.csv', 'r') 
        list = seed_file.readlines()
        for pos in list: 
            res = [float(i) for i in pos.strip('][\n').split(', ')]
            b_pos_to_generate_alternative.append(res)
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            t_field.setSFVec3f(res) 
            block_list.append(rec_node)   
        
    else: 
        # if already generated, use the previously saved positions 
        for i in b_pos_to_generate_alternative: 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            t_field.setSFVec3f(i) 
            block_list.append(rec_node)
        
def regenerate_blocks_single_source():
    global block_list
    global r_pos_to_generate
    global b_pos_to_generate
    
    for obj in block_list: 
        obj.remove()
    
    block_list = []
    
    for i in range(len(r_pos_to_generate)):
        population[i].getField('translation').setSFVec3f(r_pos_to_generate[i])
        
    if len(b_pos_to_generate) == 0: 
        for i in range(40): 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            t_field.setSFVec3f([round(random.uniform(-0.5, -0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02]) 
            block_list.append(rec_node)
            
    else: 
        # if already generated, use the previously saved positions 
        for i in b_pos_to_generate: 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            t_field.setSFVec3f(i) 
            block_list.append(rec_node)
        
def regenerate_blocks_dual_source():
    global block_list
    global r_pos_to_generate
    global b_pos_to_generate
    
    for obj in block_list: 
        obj.remove()
    
    block_list = []
    
    if len(b_pos_to_generate) == 0: 
        for i in range(len(r_pos_to_generate)):
            population[i].getField('translation').setSFVec3f(r_pos_to_generate[i])
            
    
        for i in range(20): 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            t_field.setSFVec3f([round(random.uniform(-0.5, -0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02]) 
            block_list.append(rec_node)        
          
        for i in range(20): 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
            
            t_field = rec_node.getField('translation')
            t_field.setSFVec3f([round(random.uniform(0.5, 0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02]) 
            block_list.append(rec_node)    
 
    else: 
        # if already generated, use the previously saved positions 
        for i in b_pos_to_generate: 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            t_field.setSFVec3f(i) 
            block_list.append(rec_node)
               
# creates random clustering         
def regenerate_blocks_power_law():
    global block_list
    global r_pos_to_generate
    global b_pos_to_generate
    
    for obj in block_list: 
        obj.remove()
    
    block_list = []
    
    for i in range(len(r_pos_to_generate)):
        population[i].getField('translation').setSFVec3f(r_pos_to_generate[i])
        
    x_min = 1
    alpha = 2.5
    centers = []
    
    if len(b_pos_to_generate) == 0: 

        seed_file = open('../../graph-generation/seed-11-pl.csv', 'r') 
        list = seed_file.readlines()
        for pos in list: 
            res = [float(i) for i in pos.strip('][\n').split(', ')]
            b_pos_to_generate_alternative.append(res)
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            t_field.setSFVec3f(res) 
            block_list.append(rec_node) 
            
    else: 
        # if already generated, use the previously saved positions 
        for i in b_pos_to_generate: 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            t_field.setSFVec3f(i) 
            block_list.append(rec_node)
            
    for rec_node in block_list: # set to be upright
        r_field = rec_node.getField('rotation')
        if r_field.getSFRotation() != [0, 0, -1]:
            r_field.setSFRotation([0, 0, -1])
                
def save_progress():
    # way to save total number of blocks found 
    # global k_gen_f
    global overall_f
    
    print('progress saved to csv')
    emitter.send('sim-complete'.encode('utf-8'))

def message_listener(time_step):
    # global k1_df 
    # global k2_df 
    # global k3_df
    global count 
    # global k_gen_f
    global total_found
    global found_list
    global block_list
    global collected_count 
    global fitness_scores
    global curr_size
    global population  
    global start 
    global simulation_time
    global prev_msg

    if receiver.getQueueLength()>0 and (robot.getTime() - start < simulation_time):
        message = receiver.getData().decode('utf-8')
        
        # print('incoming messages', message) 
        
        if message[0] == "$": # handles deletion of objects when grabbed
            obj_node = robot.getFromId(int(message.split("-")[1]))
            # print(obj_node)
            if obj_node is not None:
                r_node_loc = population[int(message.split("-")[0][1:])].getField('translation').getSFVec3f()
                t_field = obj_node.getField('translation')
                t_node_loc = t_field.getSFVec3f()
                
                # print(math.dist(r_node_loc, t_node_loc))
                if (math.dist(r_node_loc, t_node_loc) < 0.15): # only count if actually in range 
                    if repopulate: 
                        # will be placed somewhere random 
                        side = random.randint(0,1)
                        if side == 1:
                            t_field.setSFVec3f([round(random.uniform(-0.5, -0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02]) 
                        else: 
                            t_field.setSFVec3f([round(random.uniform(0.5, 0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02])   
                    else:
                        t_field.setSFVec3f([-0.9199,-0.92, 0.059]) 
                                            
                    if obj_node not in found_list:
                        total_found += 1
                        found_list.append(obj_node)
                        collected_count[int(message.split("-")[0][1:])] = collected_count[int(message.split("-")[0][1:])] + 1
                        msg_info = "%" + message[1:]
                        if msg_info != prev_msg: 
                            emitter.send(str(msg_info).encode('utf-8'))
                            prev_msg = msg_info 
                        print('removing object') 
 
                    
                    # total_found += 1
            receiver.nextPacket()
            
        elif 'fitness' in message:
            print('message', message) 
            fit = message.split('-')[1][7:] 
            index = message.split('-')[0][1:]
            fitness_scores[int(index)] = fit
            print('fitness scores', fitness_scores)
            
            eval_fitness(time_step)
            
            receiver.nextPacket()
            # will be generalized 
            
        elif message[0] == '*':
            new_response = message 
            if prev_msg != new_response: 
                emitter.send(str(new_response).encode('utf-8'))
                prev_msg = new_response
            receiver.nextPacket()
            
            # notify other robots 
            
            
        else: 
            receiver.nextPacket()
            
    elif (robot.getTime() - start > simulation_time and prev_msg != 'clean finish'):
    # if over time would want to reset 
        emitter.send('cleaning'.encode('utf-8'))
        
        while receiver.getQueueLength()>0:
            receiver.nextPacket()
            
        emitter.send('clean finish'.encode('utf-8'))
        prev_msg = 'clean finish'

# runs simulation for designated amount of time 
def run_seconds(t,waiting=False):
    global fitness_scores
    global updated
    global fit_update 
    global start 
    global prev_msg 
    
    n = TIME_STEP / 1000*32 # convert ms to s 
    start = robot.getTime()
    
    new_t = round(t, 1)
    
    while robot.step(TIME_STEP) != -1:
        # run robot simulation for 30 seconds (if t = 30)
        increments = TIME_STEP / 1000
        
        # if waiting:
            # if not updated:
                # message_listener(robot.getTime())
                # eval_fitness(robot.getTime())
                # continue 
            # else: 
                # break 
        
        if robot.getTime() - start > new_t: 
            message_listener(robot.getTime())
            emitter.send('return_fitness'.encode('utf-8'))
            prev_msg = 'return_fitness'
            print('requesting fitness')
            break 

        elif not waiting: 
            # constantly checking for messages from robots 
            message_listener(robot.getTime())
                         
            if total_found == len(block_list):
                # new_row = {'time step': robot.getTime(), 'fitness': 0} # this is just here to record time to finish task  
                # k1_df.append(new_row, ignore_index=True)
                emitter.send('return_fitness'.encode('utf-8'))
                prev_msg = 'return_fitness'
                print('collected all objects')
                break      
    return 
            
def update_geno_list(): 
    global fitness_scores
    global taken 
    global updated 
    global count 
                     
    # update parameters to hopefully improve performance
    fitness_scores = ["!" for i in range(len(population))]
    fit_update = False 
    print('gene pool updated', fitness_scores) 
    updated = True
    
 
   
# fitness function for each individual robot 
def eval_fitness(time_step):
    global fitness_scores 
    global fit_update
    global updated
            
    if '!' not in fitness_scores: 
        # receiver.nextPacket()
        print('will update gene pool --')
        fit_update = True 
        update_geno_list()
  
def run_optimization():
    global updated
    global simulation_time 
    global total_found 
    global overall_f
    global found_list
    global r_pos_to_generate
    global overall_columns
    global curr_size 
    global population 
    global curr_trial 
    global assessing 

    # generate_robot_central(5)
    # generate_robot_central(robot_population_sizes[0])
    # regenerate_environment(0.2)
    # run_seconds(simulation_time)
    # print('new generation beginning')
    # run_seconds(5, True) # is waiting until got genotypes
    for size in robot_population_sizes: 
        curr_size = size 
        
        r_pos_to_generate = []
        generate_robot_central(size)
        curr_trial = 0 
        
        if assessing and curr_trial % 2 == 0:
            # regenerate_environment(0.2)
            regenerate_blocks_power_law()
            # regenerate_environment_alternate(0.2) 
        elif assessing and curr_trial % 2 != 0: 
            regenerate_environment_alternate(0.2)    
        else: 
            # regenerate_environment(0.2)
            regenerate_blocks_power_law()
        # regenerate_blocks_single_source()
        # regenerate_blocks_dual_source()
        
        for rec_node in population: 
            r_field = rec_node.getField('rotation')
            if r_field.getSFRotation() != [0, 0, -1]:
                r_field.setSFRotation([0, 0, -1])
        
        for i in range(trials): 
            print('beginning new trial', i)
            for gen in range(num_generations): 
                
                updated = False     
                run_seconds(simulation_time) 
                
                # print('waiting for genotypes')
                
                # run_seconds(5, True) # is waiting until got genotypes
                
                print('found genotypes')
                print('new generation starting -')
                
            for rec_node in population: 
                r_field = rec_node.getField('rotation')
                if r_field.getSFRotation() != [0, 0, -1]:
                    r_field.setSFRotation([0, 0, -1])
            print('here')
            overall_f.write(str(i) + ',' + str(robot.getTime()) + ',' + str(total_found) + ',' + str(size)+ ',invert-ant' + '\n')   
            overall_f.close()
            overall_f = open('../../graph-generation/collection-data/overall-ant-info.csv', 'a')  
            # new_row = {'trial': i,'time': simulation_time*num_generations, 'objects retrieved': total_found}
            print('items collected', total_found)
            # overall_df = pd.concat([overall_df, pd.DataFrame([new_row])], ignore_index = True)
            # restore_positions()  
            curr_trial = i + 1
            if assessing and curr_trial % 2 == 0:
                # regenerate_environment(0.2)
                regenerate_blocks_power_law()
                # regenerate_environment_alternate(0.2)
                emitter.send('trial_complete-'.encode('utf-8')) 
            elif assessing and curr_trial % 2 != 0: 
                regenerate_environment_alternate(0.2)
                emitter.send('trial_complete'.encode('utf-8'))     
            else: 
                # regenerate_environment(0.2)
                regenerate_blocks_power_law()
                emitter.send('trial_complete-'.encode('utf-8')) 
            # regenerate_blocks_power_law()
            # regenerate_blocks_single_source()
            # regenerate_blocks_dual_source()
            total_found = 0 
            found_list = [] 
            index = 0 
            # emitter.send('trial_complete'.encode('utf-8')) 
                
    overall_f.close()
    
    return 
   
        
            
def main(): 
    # restore_positions()
    # initialize_genotypes()
    run_optimization()
    save_progress()
  
         
main()
                    
            
            
