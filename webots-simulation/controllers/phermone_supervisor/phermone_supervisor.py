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

# sets up csv for reference 
# k_gen_f = open('gen-crw-info.csv', 'w')
# k_gen_f.write('time step' + ',fitness' + ',xpos' + ',ypos' + ',num col')
# k_gen_f.close()

# k_gen_f = open('gen-crw-info.csv', 'a')
# k_gen_df = pd.DataFrame(columns = ['time step', 'fitness', 'xpos', 'ypos', 'num col'])

# Agent File Initialization 
strategy_f = open("../inverted_ant/ant-info.csv", 'w')
strategy_f.write('agent id,'+ 'time step,' +' time since last block' + ',size' + ',collisions'+ '\n')
strategy_f.close()

# Global File Initialization 
overall_f = open('overall-ant-info.csv', 'w') 
overall_f.write('trial' + ',time' + ',objects retrieved' + ',size'+ '\n')
overall_f.close()

overall_f = open('overall-ant-info.csv', 'a') 
# overall_df = pd.DataFrame(columns = ['trial', 'time', 'objects retrieved'])

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance

# get info from this def 
# k1 = robot.getFromDef("khepera")
# k2 = robot.getFromDef("khepera2")
# k3 = robot.getFromDef("khepera3")

# emitter to send info to robots 
emitter = robot.getDevice("emitter")
emitter.setChannel(1)

# set receiver to receive info 
receiver = robot.getDevice("receiver") 
receiver.enable(TIME_STEP)
receiver.setChannel(2) 

num_generations = 10

global population 
population = []
# population = [k1, k2, k3]

global initial_genotypes 
initial_genotypes = []

global k1_fitness
global k2_fitness
global k3_fitness
global fitness_scores
fitness_scores = []
global pop_genotypes 
pop_genotypes = []

total_collected = 0 

taken = False # getting second child assigned 

gene_list = ['control speed 10', 'detection threshold 1000', 'time switch 550']

total_found = 0

updated = False

fit_update = False 

simulation_time = 30

count = 0

trials = 15

found_list = []
 
block_list = []

arena_area = robot.getFromDef("arena")

robot_population_sizes = [5, 10, 15]

collected_count = []

r_pos_to_generate = []

overall_columns = ['trial','time', 'objects retrieved', 'size']

curr_size = 5

def generate_robot_central(num_robots):
    global fitness_scores 
    global collected_count 
    global population
    global overall_columns 
    global curr_df
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
        rootChildrenField.importMFNode(-1, '../supervisor_controller/robots/robot-ant.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        pos = [round(random.uniform(0.25, -0.25),2), round(random.uniform(0.25, -0.25) ,2), 0.02]
        r_pos_to_generate.append(pos)
        t_field.setSFVec3f(pos)
        
        # sets up metrics 
        fitness_scores.append("!")
        collected_count.append(0)
        population.append(rec_node)
        
        curr_df = open('robot-info-' + str(num_robots) + '.csv', 'a')
        # k2_f = open('robot-2-info.csv', 'a')
        # k3_f = open('robot-3-info.csv', 'a') 
        

def regenerate_environment(block_dist):
    # creates a equally distributed set of blocks 
    # avoiding areas where a robot is already present 
    global block_list
    global population 
    global r_pos_to_generate
    
    for obj in block_list: 
        obj.remove()
    
    block_list = []
    
    for i in range(len(r_pos_to_generate)):
        population[i].getField('translation').setSFVec3f(r_pos_to_generate[i])
    
    # floor_size = arena_area.getField('floorSize')
    # print('arena size --', floor_size.getSFVec2f()) 
    # tile_size = arena_area.getField('floorTileSize')
    # print('tile size --', tile_size.getSFVec2f()) 
    
    # generates block on opposite sides of arena (randomly generated) 
    for i in range(10): 
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../supervisor_controller/cylinder-obj.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        t_field.setSFVec3f([round(random.uniform(0.9, -0.9),2), round(random.uniform(0.3, 0.85),2), 0.02]) 
        block_list.append(rec_node)
    
    for i in range(10): 
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../supervisor_controller/cylinder-obj.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        t_field.setSFVec3f([round(random.uniform(0.9, -0.9),2), round(random.uniform(-1, 0.23),2), 0.02]) 
        block_list.append(rec_node)
        
        
def generate_blocks_single_source():

    for i in range(40): 
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../supervisor_controller/cylinder-obj.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        t_field.setSFVec3f([round(random.uniform(0.9, -0.9),2), round(random.uniform(-1, 0.23),2), 0.02]) 
        block_list.append(rec_node)
        

# def initialize_genotypes(size):
    # global initial_genotypes
    # global gene_list 
    # initial_geno_txt = open('initial_genotype.txt', 'w')
    
    # lines = []
    # for r in range(size):
        # new_geno = create_individal_genotype(gene_list)
        # initial_genotypes.append(new_geno)
        

# def restore_positions():
        # clearing messages between each trial 
    # while receiver.getQueueLength()>0:
        # message = receiver.getData().decode('utf-8')
        # receiver.nextPacket()
        
    # robot.simulationReset()   
    
    # coordinates = [[-0.115, 0, 0.0045], [0.2445, 0, 0.0045], [0.6045, 0, 0.0045]]
    # for r in range(len(population)): 
        # population[r].restartController()
        # r_field = population[r].getField('translation')
        # r_field.setSFVec3f(coordinates[r])
    # initialize_genotypes()
    
def save_progress():
    # way to save total number of blocks found 
    # global k_gen_f
    global overall_f
    
    # new_row = {'time': simulation_time*num_generations, 'objects retrieved': total_found}
    # overall_df = pd.concat([overall_df, pd.DataFrame([new_row])], ignore_index=True)
    # overall_f.write('time:' + str(simulation_time*num_generations) + ',objects retrieved:' + str(total_found))
    # overall_f.close()
    # k_gen_f.close()
    # k_gen_df.to_csv('k_gen_results.csv')
    # overall_df.to_csv('overall_results.csv')
    
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

    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
        
        # print('incoming messages', message) 
        
        if message[0] == "$": # handles deletion of objects when grabbed
            # collected_count[int(message[1])] = collected_count[int(message[1])] + 1
            # print('removing object')
            # message = message[1:]
            # print(message)
            obj_node = robot.getFromId(int(message.split("-")[1]))
            # print(obj_node)
            if obj_node is not None:
                r_node_loc = population[int(message.split("-")[0][1:])].getField('translation').getSFVec3f()
                t_field = obj_node.getField('translation')
                t_node_loc = t_field.getSFVec3f()
                
                # print(math.dist(r_node_loc, t_node_loc))
                if (math.dist(r_node_loc, t_node_loc) < 0.15): # only count if actually in range 
                    t_field.setSFVec3f([-0.9199,-0.92, 0.059]) 
                    # obj_node.remove()
                    # remove redundant requests 
                    if obj_node not in found_list:
                        total_found += 1
                        found_list.append(obj_node)
                        collected_count[int(message.split("-")[0][1:])] = collected_count[int(message.split("-")[0][1:])] + 1
                        msg_info = "%" + message[1:]
                        emitter.send(str(msg_info).encode('utf-8'))
                        print('removing object') 
 
                    
                    # total_found += 1
            receiver.nextPacket()
            
        elif 'fitness' in message:
            print('message', message) 
            fit = message.split('-')[1][7:] 
            index = message.split('-')[0][1:]
            fitness_scores[int(index)] = fit
            print('fitness scores', fitness_scores)
            
            
            curr_df.write('agent id,' + str(index) + ',time step, ' + str(robot.getTime()) + ',fitness,' + str(fit) + ',xpos,' + str(population[int(index)].getPosition()[0]) + ',ypos,' + str(population[int(index)].getPosition()[1]) + ',num col,' + str(collected_count[int(index)]) + ',genotype,'+ '\n')
            curr_df.close()
            curr_df = open('robot-info-' + str(curr_size) + '.csv', 'a')
            
            receiver.nextPacket()
            # will be generalized 
            
        elif message[0] == '*':
            new_reponse = message 
            emitter.send(str(new_reponse).encode('utf-8'))
            receiver.nextPacket()
            
            # notify other robots 
            
            
        else: 
            receiver.nextPacket()

# runs simulation for designated amount of time 
def run_seconds(t,waiting=False):
    global pop_genotypes
    global fitness_scores
    global updated
    global fit_update 
    
    n = TIME_STEP / 1000*32 # convert ms to s 
    start = robot.getTime()
    
    new_t = round(t, 1)
    
    while robot.step(TIME_STEP) != -1:
        # run robot simulation for 30 seconds (if t = 30)
        increments = TIME_STEP / 1000
        
        if waiting:
            if not updated:
                message_listener(robot.getTime())
                eval_fitness(robot.getTime())
                continue 
            else: 
                break 
        
        elif robot.getTime() - start > new_t: 
            emitter.send('return_fitness'.encode('utf-8'))
            print('requesting fitness')
            break 
                
        # if reset_position: 
            # restore_positions()
        
        elif not waiting: 
            # constantly checking for messages from robots 
            message_listener(robot.getTime())
                         
            if total_found == len(block_list):
                # new_row = {'time step': robot.getTime(), 'fitness': 0} # this is just here to record time to finish task  
                # k1_df.append(new_row, ignore_index=True)
                emitter.send('return_fitness'.encode('utf-8'))
                print('collected all objects')
                break      
    return 
            
def update_geno_list(genotype_list): 
    global fitness_scores
    global pop_genotypes 
    global gene_list
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
    global pop_genotypes 
    global fitness_scores 
    global fit_update
    global updated
    
    # print('fitness scores ', fitness_scores)
            
    if '!' not in fitness_scores: 
        # receiver.nextPacket()
        print('will update gene pool --')
        fit_update = True 
        update_geno_list(pop_genotypes)
          
    
def run_optimization():
    global pop_genotypes 
    global gene_list 
    global updated
    global simulation_time 
    global total_found 
    global overall_f
    global found_list
    global r_pos_to_generate
    global overall_columns
    global curr_size 
    
    # initialize genotypes 
    # will be same genotype as normal (for comparison purposes) 
    
    # generate_robot_central(5)
    generate_robot_central(robot_population_sizes[0])
    regenerate_environment(0.2)
    run_seconds(simulation_time)
    print('new generation beginning')
    run_seconds(5, True) # is waiting until got genotypes
    
    # index = 0 
    # for i in range(3):
        # genotype = create_individal_genotype(gene_list)
        # print('genotype', i, genotype)
        # genotype = initial_genotypes[index]
        # pop_genotypes.append(genotype)
        # emitter.send(str("#" + str(i) + str(genotype)).encode('utf-8'))
        # index +=1 
    
    # regenerate_environment(0.2)   
    # run_seconds(simulation_time) # runs generation for that given amount of time  
    # print('new generation beginning')
    # run_seconds(5, True) # is waiting until got genotypes
    
    # regenerate_environment(0.2) 
    for size in robot_population_sizes: 
        curr_size = size 
    
        # initialize_genotypes(size)
                # creates a csv specific to the robot 
        curr_df = open('robot-info-' + str(size) + '.csv', 'w')
        # k2_f = open('robot-2-info.csv', 'w')
        # k3_f = open('robot-3-info.csv', 'w')
        
        curr_df.write(str(overall_columns)+ '\n')
        # k2_f.write(str(columns))
        # k3_f.write(str(columns))
        
        curr_df.close()
        # k2_f.close()
        # k3_f.close()
        
        r_pos_to_generate = []
        generate_robot_central(size)
        
        for i in range(trials): 
            print('beginning new trial', i)
            for gen in range(num_generations-1): 
                
                updated = False     
                run_seconds(simulation_time) 
                
                print('waiting for genotypes')
                
                run_seconds(5, True) # is waiting until got genotypes
                
                print('found genotypes')
                print('new generation starting -')
            
            overall_f.write('trial,' + str(i) + ',time,' + str(robot.getTime()) + ',objects retrieved,' + str(total_found) + ',size,' + str(size))   
            overall_f.close()
            overall_f = open('overall-ant-info.csv', 'a')  
            # new_row = {'trial': i,'time': simulation_time*num_generations, 'objects retrieved': total_found}
            print('items collected', total_found)
            # overall_df = pd.concat([overall_df, pd.DataFrame([new_row])], ignore_index = True)
            # restore_positions()  
            regenerate_environment(0.2) 
            total_found = 0 
            found_list = [] 
            index = 0 
            emitter.send('trial_complete'.encode('utf-8'))
            # for i in range(3):
                # genotype = create_individal_genotype(gene_list)
                # print('genotype', i, genotype)
                # genotype = initial_genotypes[index]
                # pop_genotypes.append(genotype)
                # emitter.send(str("#" + str(i) + str(genotype)).encode('utf-8'))
                # index +=1  
        curr_df.close()
                
    overall_f.close()
    
    return 
   
        
            
def main(): 
    # restore_positions()
    # initialize_genotypes()
    run_optimization()
    save_progress()
  
         
main()
                    
            
            