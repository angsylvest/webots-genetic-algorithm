from controller import Supervisor, Node, Keyboard, Emitter, Receiver, Field
import statistics 
import pandas as pd
import random
import math 

"""
Main supervisor base 
Optimization algorithm - Collaboration-oriented 
Angel Sylvester 2022
"""

# sets up csv for reference 
k_gen_df = pd.DataFrame(columns = ['time step', 'fitness', 'xpos', 'ypos', 'num col'])
overall_df = pd.DataFrame(columns = ['trial', 'time', 'objects retrieved'])

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance

# get info from this def 
k1 = robot.getFromDef("khepera")
k2 = robot.getFromDef("khepera2")
k3 = robot.getFromDef("khepera3")

# emitter to send info to robots 
emitter = robot.getDevice("emitter")
emitter.setChannel(1)

# set receiver to receive info 
receiver = robot.getDevice("receiver") 
receiver.enable(TIME_STEP)
receiver.setChannel(2) 

num_generations = 10
population = [k1, k2, k3]

global initial_genotypes 
initial_genotypes = []

global k1_fitness
global k2_fitness
global k3_fitness
global fitness_scores
fitness_scores = ["!","!","!"]
global pop_genotypes 
pop_genotypes = []

total_collected = 0 

taken = False # getting second child assigned 

gene_list = ['control speed 10', 'detection threshold 1000', 'time switch 550']

total_found = 0

updated = False

fit_update = False 

simulation_time = 15

count = 0

trials = 15

found_list = []
 
block_list = []

arena_area = robot.getFromDef("arena")

def regenerate_environment(block_dist):
    # creates a equally distributed set of blocks 
    # avoiding areas where a robot is already present 
    global block_list
    for obj in block_list: 
        obj.remove()
    
    block_list = []
    
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

def initialize_genotypes():
    global initial_genotypes
    with open('initial_genotype.txt') as f:
        for line in f: 
            initial_genotypes.append(line)

def restore_positions():
        # clearing messages between each trial 
    while receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
        receiver.nextPacket()
        
    # robot.simulationReset()   
    
    coordinates = [[-0.115, 0, 0.0045], [0.2445, 0, 0.0045], [0.6045, 0, 0.0045]]
    for r in range(len(population)): 
        population[r].restartController()
        r_field = population[r].getField('translation')
        r_field.setSFVec3f(coordinates[r])
    initialize_genotypes()
    
def save_progress():
    # way to save total number of blocks found 
    global k_gen_df
    global overall_df
    
    new_row = {'time': simulation_time*num_generations, 'objects retrieved': total_found}
    overall_df = pd.concat([overall_df, pd.DataFrame([new_row])], ignore_index=True)
    k_gen_df.to_csv('k_gen_results.csv')
    overall_df.to_csv('overall_results.csv')
    
    print('progress saved to csv')
    emitter.send('sim-complete'.encode('utf-8'))

def message_listener(time_step):
    global k1_df 
    global k2_df 
    global k3_df
    global count 
    global k_gen_df
    global total_found
    global found_list
    global block_list

    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
        
        print('incoming messages', message) 
        
        if message[0] == "$": # handles deletion of objects when grabbed
            # collected_count[int(message[1])] = collected_count[int(message[1])] + 1
            print('removing object')
            # message = message[1:]
            print(message)
            obj_node = robot.getFromId(int(message[2:]))
            print(obj_node)
            if obj_node is not None:
                r_node_loc = population[int(message[1])].getField('translation').getSFVec3f()
                t_field = obj_node.getField('translation')
                t_node_loc = t_field.getSFVec3f()
                
                print(math.dist(r_node_loc, t_node_loc))
                if (math.dist(r_node_loc, t_node_loc) < 0.15): # only count if actually in range 
                    t_field.setSFVec3f([-0.9199,-0.92, 0.059]) 
                    # obj_node.remove()
                    # remove redundant requests 
                    if obj_node not in found_list:
                        total_found += 1
                        found_list.append(obj_node)
                    
                    # total_found += 1
            receiver.nextPacket()
            
        elif 'k-fitness' in message:
            print('message', message)
            k3_fitness = int(message[9:])
            fitness_scores[count] = k3_fitness
            count += 1
            
            new_row = {'time step': time_step, 'fitness': k3_fitness}
            k_gen_df = pd.concat([k_gen_df, pd.DataFrame([new_row])], ignore_index=True)
            print('k fitness', k3_fitness)
            
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
    fitness_scores = ["!","!","!"]
    count = 0 
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
    global overall_df
    global found_list
    
    # initialize genotypes 
    # will be same genotype as normal (for comparison purposes) 
    
    index = 0 
    for i in range(3):
        # genotype = create_individal_genotype(gene_list)
        # print('genotype', i, genotype)
        genotype = initial_genotypes[index]
        pop_genotypes.append(genotype)
        emitter.send(str("#2" + str(genotype)).encode('utf-8'))
        index +=1 
    
    regenerate_environment(0.2)   
    run_seconds(simulation_time) # runs generation for that given amount of time  
    print('new generation beginning')
    run_seconds(5, True) # is waiting until got genotypes
    
    # regenerate_environment(0.2) 
    for i in range(trials): 
        print('beginning new trial', i)
        for gen in range(num_generations-1): 
            
            updated = False     
            run_seconds(simulation_time) 
            
            print('waiting for genotypes')
            
            run_seconds(5, True) # is waiting until got genotypes
            
            print('found genotypes')
            print('new generation starting -')
            
        new_row = {'trial': i,'time': simulation_time*num_generations, 'objects retrieved': total_found}
        print('items collected', total_found)
        overall_df = pd.concat([overall_df, pd.DataFrame([new_row])], ignore_index = True)
        # restore_positions()  
        regenerate_environment(0.2) 
        total_found = 0 
        found_list = []     
    return 
   
        
            
def main(): 
    # restore_positions()
    initialize_genotypes()
    run_optimization()
    save_progress()
  
         
main()
                    
            
            