from controller import Supervisor, Node, Keyboard, Emitter, Receiver
import statistics 
import math 
import pandas as pd
from robot_pop import * 
from graph_generator import * 

"""
Main supervisor base 
Optimization algorithm - Collaboration-oriented 
Angel Sylvester 2022
"""

# sets up csv for reference 
k1_df = pd.DataFrame(columns = ['agent id' ,'time step', 'fitness', 'xpos', 'ypos'])
k2_df = pd.DataFrame(columns = ['agent id', 'time step', 'fitness', 'xpos', 'ypos'])
k3_df = pd.DataFrame(columns = ['agent id', 'time step', 'fitness', 'xpos', 'ypos'])
## add more robot dfs here (be sure to keep num consistent) 

global df_list
df_list = [k1_df, k2_df, k3_df]

overall_df = pd.DataFrame(columns = ['time', 'objects retrieved'])

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance

# get info from this def 
k1 = robot.getFromDef("khepera")
k2 = robot.getFromDef("khepera2")
k3 = robot.getFromDef("khepera3")
## add more robots here (if defined, be sure to keep num consistent) 

population = [k1, k2, k3] # add population here 

# translation_field_1 = k1.getField('translation')
# rotation_field_1 = k1.getField('rotation')
# translation_field_2 = k2.getField('translation')
# rotation_field_2 = k2.getField('rotation')
# translation_field_3 = k3.getField('translation')
# rotation_field_3 = k3.getField('rotation')

# emitter to send info to robots 
emitter = robot.getDevice("emitter")
emitter.setChannel(1)

# set receiver to receive info 
receiver = robot.getDevice("receiver") 
receiver.enable(TIME_STEP)
receiver.setChannel(2) 

num_generations = 2

global initial_genotypes 
initial_genotypes = []

global k1_fitness
global k2_fitness
global k3_fitness
global fitness_scores
fitness_scores = ["!","!","!"]
global pop_genotypes 
pop_genotypes = []

global taken 
taken = False # getting second child assigned 

global gene_list 
gene_list = ['control speed 10', 'detection threshold 1000', 'time switch 550']

global total_found 
total_found = 0

global updated # regarding genepool 
updated = False

global fit_update
fit_update = False 

global simulation_time
simulation_time = 1

def initialize_genotypes():
    global initial_genotypes
    with open('initial_genotype.txt') as f:
        for line in f: 
            initial_genotypes.append(line)

def restore_positions():
    pass 
    
def find_nearest_robot_genotype(r_index):
    global population 
    closest_neigh = " "
    curr_robot = population[r_index]
    curr_dist = 400 # arbitrary value 
    curr_fitness = fitness_scores[r_index]
    other_fitness = 0
    
    curr_pos = [curr_robot.getPosition()[0], curr_robot.getPosition()[1]]
    other_index = r_index
    
    for i in range(len(population)):
        if (i != r_index): 
            other_pos = [population[i].getPosition()[0], population[i].getPosition()[0]]
            dis = math.dist(curr_pos, other_pos)
            if closest_neigh == " ":
                closest_neigh = population[i].getId()
                curr_dist = dis
                other_fitness = fitness_scores[i]
                other_index = i
            elif dis < curr_dist: 
                closest_neigh = population[i]
                curr_dist = dis 
                other_fitness = fitness_scores[i]
                other_index = i
    # print('found closest neighbor', closest_neigh)
    # use emitter to send genotype to corresponding robot if fitness is better and if nearby 
    if type(curr_fitness) == 'int' and type (other_fitness) == 'int' and other_fitness > (curr_fitness + 1): 
        emitter.send(str("#"+ str(r_index) + str(pop_genotypes[i])).encode('utf-8'))
        # will probably increase fitness between these two for communciation
                
    
def save_progress():
    # way to save total number of blocks found 
    global overall_df
    global df_list 
    global k1_df
    global k2_df 
    global k3_df
   
    
    generate_fitness_csvs(df_list)
    generate_fitness("summary-fitness.csv")
 
    
    # csv_list = []
    
    new_row = {'time': simulation_time*num_generations, 'objects retrieved': total_found}
    overall_df = pd.concat([overall_df, pd.DataFrame([new_row])], ignore_index = True)
            
    # k1_df.to_csv('k1_results.csv')
    # k2_df.to_csv('k2_results.csv') 
    # k3_df.to_csv('k3_results.csv') 
    
    overall_df.to_csv('overall_results.csv')
    
    print('progress saved to csv')
    emitter.send('sim-complete'.encode('utf-8'))

def message_listener(time_step):
    global k1_df 
    global k2_df 
    global k3_df
    global total_found 
    global df_list 

    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
        
        print('incoming messages', message) 
        
        if message[0] == "$": # handles deletion of objects when grabbed
            message = int(message[1:])
            receiver.nextPacket()
            print('removing object')
            obj_node = robot.getFromId(message)
            print(obj_node)
            if obj_node is not None: 
                obj_node.remove()
                total_found += 1
            
        elif 'k1-fitness' in message: 
            k1_fitness = int(message[10:])
            fitness_scores[0] = k1_fitness
            
            new_row = {'agent id': 1, 'time step': time_step, 'fitness': k1_fitness, 'xpos': k1.getPosition()[0], 'ypos': k1.getPosition()[1]}
            k1_df = pd.concat([k1_df, pd.DataFrame([new_row])], ignore_index=True)
            df_list[0] = k1_df
            
            print('k1 fitness', k1_fitness)
            
            receiver.nextPacket()
            
        elif 'k2-fitness' in message: 
            k2_fitness = int(message[10:])
            fitness_scores[1] = k2_fitness
            
            new_row = {'agent id': 2,'time step': time_step, 'fitness': k2_fitness, 'xpos': k2.getPosition()[0], 'ypos': k2.getPosition()[1]}
            k2_df = pd.concat([k2_df, pd.DataFrame([new_row])], ignore_index = True)
            df_list[1] = k2_df
            
            print('k2 fitness', k2_fitness)
            
            receiver.nextPacket()
            
        elif 'k3-fitness' in message:
            k3_fitness = int(message[10:])
            fitness_scores[2] = k3_fitness
            
            new_row = {'agent id': 3,'time step': time_step, 'fitness': k3_fitness, 'xpos': k3.getPosition()[0], 'ypos': k3.getPosition()[1]}
            k3_df = pd.concat([k3_df, pd.DataFrame([new_row])], ignore_index = True)
            df_list[2] = k3_df
            
            print('k3 fitness', k3_fitness)
            
            receiver.nextPacket()
            
        elif 'encounter' in message: 
            print('robot found -- checking genotype') 
            robo_index = int(message.split('-')[0])
            new_geno = find_nearest_robot_genotype(robo_index)
            
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
            # elif not fit_update: 
                # continue  
            else: 
                break 
        
        elif robot.getTime() - start > new_t: 
            emitter.send('return_fitness'.encode('utf-8'))
            print('requesting fitness')
            # message_listener(robot.getTime())
            # fit_update = False
            # eval_fitness(robot.getTime())
            
            # if fit_update:
            break 
                
        # if reset_position: 
            # restore_positions()
        
        elif not waiting: 
            # constantly checking for messages from robots 
            message_listener(robot.getTime())
                         
            if total_found == 8:
                # new_row = {'time step': robot.getTime(), 'fitness': 0} # this is just here to record time to finish task  
                # k1_df.append(new_row, ignore_index=True)
                print('collected all objects')
                break      
    return 
            
def update_geno_list(genotype_list): 
    global fitness_scores
    global pop_genotypes 
    global gene_list
    global taken 
    global updated 
        
    if max(fitness_scores) == 0:
        # update all genotypes 
        pop_genotypes = []
        
        for i in range(len(population)):
            g = create_individal_genotype(gene_list)
            pop_genotypes.append(g)
        
        # g1 = create_individal_genotype(gene_list)
        # g2 = create_individal_genotype(gene_list)
        # g3 = create_individal_genotype(gene_list)
        # pop_genotypes.append(g1)
        # pop_genotypes.append(g2)
        # pop_genotypes.append(g3)

    else: 
        # only update lowest two genotypes 
        max_index = fitness_scores.index(max(fitness_scores))
        max_geno = pop_genotypes[max_index]
        cp_genotypes = pop_genotypes.copy()
        
        cp_genotypes.remove(pop_genotypes[max_index])
        child = reproduce(cp_genotypes[0], pop_genotypes[max_index])
        other_child = reproduce(pop_genotypes[max_index], cp_genotypes[1])
        
        
        for i in range(len(fitness_scores)):
            if i != max_index and not taken: 
                pop_genotypes[0] = child
                taken = True 
            elif i != max_index and taken: 
                pop_genotypes[1] = other_child
                taken = False 
        
        # replace genotypes of one of parents 
                     
    # update parameters to hopefully improve performance
    fitness_scores = ["!","!","!"]
    fit_update = False 
    print('gene pool updated') 
    updated = True
    
 
    

# fitness function for each individual robot 
def eval_fitness(time_step):
    global pop_genotypes 
    global fitness_scores 
    global fit_update
    global k1_df 
    global k2_df 
    global k3_df
            
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
    
    # initialize genotypes 
    # will be same genotype as normal (for comparison purposes) 
    
    index = 0 
    for i in range(len(population)):
        # genotype = create_individal_genotype(gene_list)
        # print('genotype', i, genotype)
        genotype = initial_genotypes[i]
        pop_genotypes.append(genotype)
        emitter.send(str("#"+ str(index) + str(genotype)).encode('utf-8'))
        index +=1 
        
    run_seconds(simulation_time) # runs generation for that given amount of time  
    print('new generation beginning')
    run_seconds(5, True) # is waiting until got genotypes
    
    
    for gen in range(num_generations-1): 
        
        # pop_fitness = [] 
        
        # send relevant genotypes to each robot, handler
        updated = False 
        
        index = 0 
        for i in range(len(population)):
            emitter.send(str("#"+ str(index) + str(pop_genotypes[index])).encode('utf-8'))
            index +=1 
            
        run_seconds(simulation_time) 
        
        print('waiting for genotypes')
        
        run_seconds(5, True) # is waiting until got genotypes
        
        print('found genotypes')
        print('new generation starting -')
                       
    return 
   
        
            
def main(): 
    restore_positions()
    initialize_genotypes()
    run_optimization()
    save_progress()
    # translation_field.setSFVec3f([0,0,0]) # reset robot position
    # rotation_field.setSFRotation([0, 0, 1, 0])
    # khepera_node.resetPhysics()
  
         
main()
                    
            
            