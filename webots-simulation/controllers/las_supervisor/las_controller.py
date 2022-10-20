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
overall_f = open('../../graph-generation/collection-data/overall-las-info.csv', 'w') 
overall_f.write('trial' + ',time' + ',objects retrieved'+ ',size'+ '\n')
overall_f.close()

overall_f = open('overall-las-info.csv', 'a') 

# individual robot collision counts 
strategy_f = open("../../graph-generation/collision-data/las-info.csv", 'w')
strategy_f.write('agent id,'+ 'time step,' + 'time since last block' + ',size' + ',collisions'+ '\n')
strategy_f.close()

# set-up robot
TIME_STEP = 32
robot = Supervisor()  # create Supervisor instance
emitter = robot.getDevice("emitter")
emitter.setChannel(1)
receiver = robot.getDevice("receiver") 
receiver.enable(TIME_STEP)
receiver.setChannel(2) 
arena_area = robot.getFromDef("arena")

# set up timing so consistent with ga 
num_generations = 10
simulation_time = 15
trials = 30
curr_size = 5
robot_population_sizes = [5, 10, 15]

# sim statistics 
population = []
initial_genotypes = []
fitness_scores = []
pop_genotypes = []
total_found = 0
found_list = []
block_list = []
collected_count = []
r_pos_to_generate = []

# thresholds that impacts events in sim 
taken = False # getting second child assigned 
updated = False
fit_update = False 
count = 0




def generate_robot_central(num_robots):
    global fitness_scores 
    global collected_count 
    global population
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
        rootChildrenField.importMFNode(-1, 'robots/robot-las.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        pos = [round(random.uniform(0.25, -0.25),2), round(random.uniform(0.25, -0.25) ,2), 0.02]
        r_pos_to_generate.append(pos)
        t_field.setSFVec3f(pos)
        
        # sets up metrics 
        fitness_scores.append("!")
        collected_count.append(0)
        population.append(rec_node)


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
        rootChildrenField.importMFNode(-1, 'cylinder-obj.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        t_field.setSFVec3f([round(random.uniform(0.9, -0.9),2), round(random.uniform(0.3, 0.85),2), 0.02]) 
        block_list.append(rec_node)
    
    for i in range(10): 
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, 'cylinder-obj.wbo') 
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
    overall_f.close()
    
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
    global curr_size 
    global population

    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
        
        # print('incoming messages', message) 
        
        if message[0] == "$": # handles deletion of objects when grabbed
            # collected_count[int(message[1])] = collected_count[int(message[1])] + 1
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
            fit = message.split('-')[1][7:] 
            index = message.split('-')[0][1:]
            fitness_scores[int(index)] = fit
            receiver.nextPacket()
            pass # will be generalized 
            
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
    print('gene pool updated') 
    updated = True
    
 
   
# fitness function for each individual robot 
def eval_fitness(time_step):
    global pop_genotypes 
    global fitness_scores 
    global fit_update
    global updated
            
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
    global curr_size
    
    # initialize genotypes 
    # will be same genotype as normal (for comparison purposes) 
    
    generate_robot_central(robot_population_sizes[0])
    regenerate_environment(0.2) 
    run_seconds(simulation_time)
    print('new generation beginning')
    run_seconds(5, True) # is waiting until got genotypes
    
    # regenerate_environment(0.2) 
    for size in robot_population_sizes: 
        curr_size = size 
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
            
            overall_f.write('trial,' + str(i) + ',time,' + str(robot.getTime()) + ',objects retrieved,' + str(total_found) + ',size,' + str(size)+ '\n')    
            overall_f.close()
            overall_f = open('../../graph-generation/collection-data/overall-las-info.csv', 'a') 
            # new_row = {'trial': i,'time': simulation_time*num_generations, 'objects retrieved': total_found}
            print('items collected', total_found)
            # overall_df = pd.concat([overall_df, pd.DataFrame([new_row])], ignore_index = True)
            # restore_positions()  
            regenerate_environment(0.2) 
            total_found = 0 
            found_list = [] 
            index = 0 
            emitter.send('trial_complete'.encode('utf-8'))
                
    overall_f.close()
    
    return 
   
        
            
def main(): 
    # restore_positions()
    # initialize_genotypes()
    run_optimization()
    # save_progress()
  
         
main()
                    
            
            
