from controller import Supervisor, Node, Keyboard, Emitter, Receiver
import statistics 
import pandas as pd
from robot_pop import * 

"""
Main supervisor base 
Optimization algorithm - Collaboration-oriented 
Angel Sylvester 2022
"""

# sets up csv for reference 
k1_df = pd.DataFrame(columns = ['genotype', 'fitness'])
k2_df = pd.DataFrame(columns = ['genotype', 'fitness'])
k3_df = pd.DataFrame(columns = ['genotype', 'fitness'])

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance

# get info from this def 
k1 = robot.getFromDef("khepera")
k2 = robot.getFromDef("khepera2")
k3 = robot.getFromDef("khepera3")

translation_field_1 = k1.getField('translation')
rotation_field_1 = k1.getField('rotation')
translation_field_2 = k2.getField('translation')
rotation_field_2 = k2.getField('rotation')
translation_field_3 = k3.getField('translation')
rotation_field_3 = k3.getField('rotation')

# emitter to send info to robots 
emitter = robot.getDevice("emitter")
emitter.setChannel(1)

# set receiver to receive info 
receiver = robot.getDevice("receiver") 
receiver.enable(TIME_STEP)
receiver.setChannel(2) 

num_generations = 50
population = [k1, k2, k3]

global k1_fitness
global k2_fitness
global k3_fitness
global fitness_scores
fitness_scores = ["!","!","!"]
global pop_genotypes 
pop_genotypes = []

global gene_list 
gene_list = ['control speed 10', 'detection threshold 1000', 'time switch 200']

def restore_positions():
    pass 
    

# runs simulation for designated amount of time 
def run_seconds(t,waiting=False):
    global pop_genotypes
    
    n = TIME_STEP / 1000*32 # convert ms to s 
    start = robot.getTime()
    
    new_t = round(t, 1)
    
    while robot.step(TIME_STEP) != -1:
        # run robot simulation for 30 seconds (if t = 30)
        increments = TIME_STEP / 1000
        if robot.getTime() - start > new_t: 
            eval_fitness()
            break 
        # if reset_position: 
            # restore_positions()
        
        if not waiting: 
            if receiver.getQueueLength()>0:
                message = receiver.getData().decode('utf-8')
                # assuming only getting messages about removing nodes
                if message[0] == "$": # handles deletion of objects when grabbed
                    message = int(message[1:])
                    receiver.nextPacket()
                    obj_node = robot.getFromId(message)
                    if obj_node is not None: 
                        obj_node.remove()             
            
    return 
            
def update_geno_list(genotype_list): 
    global fitness_scores
    global pop_genotypes 
    global gene_list 
    
    print('fitness list', fitness_scores)
        
    if max(fitness_scores) == 0:
        # update all genotypes 
        pop_genotypes = []
        
        g1 = create_individal_genotype(gene_list)
        g2 = create_individal_genotype(gene_list)
        g3 = create_individal_genotype(gene_list)
        pop_genotypes.append(g1)
        pop_genotypes.append(g2)
        pop_genotypes.append(g3)

    else: 
        # only update lowest two genotypes 
        max_index = fitness_scores.index(max(fitness_scores))
        max_geno = pop_genotypes[max_index]
        cp_genotypes = pop_genotypes.copy()
        
        cp_genotypes.remove(pop_genotypes[max_index])
        child = reproduce(cp_genotypes[0], cp_genotypes[1])
        
        # replace genotypes of one of parents 
        min_index = fitness_scores.index(min(fitness_scores))
        pop_genotypes[min_index] = child 
                     
    # update parameters to hopefully improve performance 
    
    fitness_scores = ["!","!","!"]
    
 
    

# fitness function for each individual robot 
def eval_fitness():
    global pop_genotypes 
    global fitness_scores 
    # send_genotype()
    # run_seconds(60)
    
    # sent the same number of times each robot appears in sim
    emitter.send('return_fitness'.encode('utf-8'))
    print('requesting fitness')
    
    # fitness = 0 
    
    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
        if 'k1-fitness' in message: 
            k1_fitness = int(message[10:])
            fitness_scores[0] = k1_fitness
            
            # new_row = {'genotype': k1_geno, 'fitness': k1_fitness}
            # k1_df.append(new_row)
            
            print('k1 fitness', k1_fitness)
            
            receiver.nextPacket()
        elif 'k2-fitness' in message: 
            k2_fitness = int(message[10:])
            fitness_scores[1] = k2_fitness
            
            # new_row = {'genotype': k2_geno, 'fitness': k2_fitness}
            # k2_df.append(new_row)
            print('k2 fitness', k2_fitness)
            
            receiver.nextPacket()
        elif 'k3-fitness' in message:
            k3_fitness = int(message[10:])
            fitness_scores[2] = k3_fitness
            
            # new_row = {'genotype': k3_geno, 'fitness': k3_fitness}
            # k3_df.append(new_row)
            print('k3 fitness', k3_fitness)
            
            receiver.nextPacket()
            
    if '!' not in fitness_scores: 
        print('will update gene pool --')
        update_geno_list(pop_genotypes)
    
    
    # while robot.step(TIME_STEP) != -1: 
        # if receiver.getQueueLength() > 0: 
            # receiver.getData().decode('utf-8')
            # receiver.nextPacket()
            # r_fit = float(message)
               
    # return fitness # will eventually return fitness calculation  
    
def run_optimization():
    global pop_genotypes 
    global gene_list 
    
    # initialize genotypes 
    index = 0 
    for i in range(len(population)):
        genotype = create_individal_genotype(gene_list)
        pop_genotypes.append(genotype)
        emitter.send(str("#"+ str(index) + str(genotype)).encode('utf-8'))
        index +=1 
        
    run_seconds(2) 
    
    
    for gen in range(num_generations-1): 
        
        pop_fitness = [] 
        
        # send relevant genotypes to each robot, handler
        
        index = 0 
        for i in range(len(population)):
            emitter.send(str("#"+ str(index) + str(pop_genotypes[index])).encode('utf-8'))
            index +=1 
            
        run_seconds(2) 
        
        # for robot in population: 
            # retrieves info about robots 
            # geno_fit = eval_fitness() # placeholder for now 
            # pop_fitness.append(geno_fit)
            
        # mean_fit = statistics.mean(pop_fitness)
        # for robot in population: 
            # geno_fit = eval_fitness() # must find way to connect each robot 
            # if geno_fit < mean_fit: 
                # reproduce(robot)
                
        print('new generation starting -')
        
        
        
                
    return 
        
            
def main(): 
    

    restore_positions()
   
    run_optimization()
    # translation_field.setSFVec3f([0,0,0]) # reset robot position
    # rotation_field.setSFRotation([0, 0, 1, 0])
    # khepera_node.resetPhysics()
   

    

         
main()
                    
            
            