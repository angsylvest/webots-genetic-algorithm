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
k1_df = pd.DataFrame(columns = ['time step', 'fitness'])
k2_df = pd.DataFrame(columns = ['time step', 'fitness'])
k3_df = pd.DataFrame(columns = ['time step', 'fitness'])

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

num_generations = 1
population = [k1, k2, k3]

global initial_genotypes 
initial_genotypes = ['1110000010*0110011010001100011011110011101111111001010111010101001010101010100101010111101101011100001010101011110111100011101000010111101100100000101101000011011101010110100101011110010100010111101000010110011001001010001101010001100011110100011011101001110001100010111010111000100100101111111001011110000110001010001010011000000100011101011011110100000000101011111001111000000111001001110100101000111001100000111100010110001100001101011101011100111001011010000101101010110001100001011101111000011101101010000110011111011001000010010000011111110001000111010011011000010001001100101001101001100001011100101101101001110011100001110101111001101100001100101010010101100000100101110100110010111111001100111110001100011001001111000110011101001011000100111000010111010110011001110111101100001101101010011011010111010110110110001110100100010001100110001110101110101011000010001111000100001011111100111011101011010100111001001001011011000101001011011110110110001011111111110001110100001001110011010001110100001000110001*1010000111100011110110001011011001010111011011010010110111110011010011010101100011010010101110111100010111011101100100011110011001000111101101110010000010000000011000100111111110110001010010101101011100101000000001101011100100000001101001110010110100100111100011001010111000100011001100010110111111010010010100100001010010000000010101000011101000100000010001100000001011101011011001100010001100110110000101011101011001100111100100111010110111000110011000110111100101111001100101000101100000110100100001111100000110010101100011110100111111010011111100*', '0001011111*1001100101011010001111101110010101101001100000010011001010100000001100010000101010010010110110111000010110010110011111100001101100000000101101111001100011111000110101001010000100110101100000010100011110001111010111111111011000111111010101110100100100001001011000101111110101111010110000001011111010101000001001000111111000000000001001010001100001100101100000110111100011100100100000100100110101101000111100101001001000011101101000110000000100111001100111110110011101000110011100110100011101111100100110011101110101001001010011000010101110000110010010110011001111110001100110011001100010010100001000010100001101011011000110111100010001011110100101001101001011001101111001000111101101111000000011110111101110010000111000001010000100110001101001111010010111110001000010011000100110100011101011010110100000011110101000001101111100010111101010011010101011010100110011011111101111110011110100011110011101100111000111000011101001001010011111001011101101110101011010000100111000010100111101001010011000010110*1110001001010110001111101001110100101010010001101011111110100111101101110000011000011101000000110101110111011101001010101010101100010000101011110110111110110000011100110111111100001001000100000100100010011111110111111101100100000010100011101111011110101111101000111101011100110100001001001001001110110111000111011111011111001100001111001001101011000011110111111001100110110101111000010011001110010110000110111010100110110101011010100001111111000111110000001011100110011010110101110101110111011101111110101111100011011001010001001110001000111001111011*', '1000001011*1100100010001110110111011010000100000111000000111101101101111110111101001011100000000110101000001000010111001111011111110101000100001001101111101101110011111010100011000010110010110011100010110010111101011010010101000001000111111101110111111011000100111001010110000111110101111110011000100100001110010111011000000000101111101100111010101010100101001101111110110101100000111111110100001110000111010000000000000110111111100100101001111001111001101100101001010010100111110101111010001010100010001110011101111001000101001011100000000110010101100010100000111111110110000011110001101001000000011011110100100011000100001011110011010101001100110101001011100011100101010011100011101111001010101100001001010010010011100001111111100000010011100101101011000111110011100100000101011110110001101100000101111000101001100000011111100110011000001001000110000011101100100100000011100000110101011000111100101101110001010110100001111111010111011110000101100001110111010011111010001000000011001011001101001000100110000011*0110100111110101011111000101101111111111011110111010001010000011111000011011001001010000010111111110111000101110010001110000011011011110000001010101111001101111010101011100000100101000011111001011011100010101001000110111000010100011101001111000010000111111100000101111100001010100111000010001110000010011010100111010101101000000100001100100010100000111100000010100101100000011100100010101111111000100100001101000001000100110100110001110101110000111101000010101011001110111001001101001001100101001110010110101100111100110011111110101010010111010101110*']


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

def restore_positions():
    pass 
    
def save_progress():
    # way to save total number of blocks found 
    new_row = {'time step': 0, 'fitness': total_found}
    k1_df.append(new_row, ignore_index=True)
            
    k1_df.to_csv('k1_results.csv')
    k2_df.to_csv('k2_results.csv') 
    k3_df.to_csv('k3_results.csv') 
    
    print('progress saved to csv')
   
    
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
                eval_fitness(robot.getTime())
                continue 
            elif not fit_update: 
                continue  
            else: 
                break 
            # if '!' not in fitness_scores:
                # break 
            # else: 
                # eval_fitness(robot.getTime())
        
        elif robot.getTime() - start > new_t: 
            emitter.send('return_fitness'.encode('utf-8'))
            print('requesting fitness')
            eval_fitness(robot.getTime())
            break 
                
        # if reset_position: 
            # restore_positions()
        
        elif not waiting: 
            if receiver.getQueueLength()>0:
                message = receiver.getData().decode('utf-8')
                # assuming only getting messages about removing nodes
                if message[0] == "$": # handles deletion of objects when grabbed
                    message = int(message[1:])
                    receiver.nextPacket()
                    print('removing object')
                    obj_node = robot.getFromId(message)
                    print(obj_node)
                    if obj_node is not None: 
                        obj_node.remove()    
            if total_found == 8:
                new_row = {'time step': robot.getTime(), 'fitness': 0} # this is just here to record time to finish task  
                k1_df.append(new_row, ignore_index=True)
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
    # send_genotype()
    # run_seconds(60)
    
    # sent the same number of times each robot appears in sim
    # emitter.send('return_fitness'.encode('utf-8'))
    # print('requesting fitness')
    
    # fitness = 0 
    
    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
        print('incoming messages', message)
        if 'k1-fitness' in message: 
            k1_fitness = int(message[10:])
            fitness_scores[0] = k1_fitness
            
            new_row = {'time step': time_step, 'fitness': k1_fitness}
            k1_df.append(new_row, ignore_index=True)
            
            print('k1 fitness', k1_fitness)
            
            receiver.nextPacket()
        elif 'k2-fitness' in message: 
            k2_fitness = int(message[10:])
            fitness_scores[1] = k2_fitness
            
            new_row = {'time step': time_step, 'fitness': k2_fitness}
            k2_df.append(new_row, ignore_index=True)
            print('k2 fitness', k2_fitness)
            
            receiver.nextPacket()
        elif 'k3-fitness' in message:
            k3_fitness = int(message[10:])
            fitness_scores[2] = k3_fitness
            
            new_row = {'time step': time_step, 'fitness': k3_fitness}
            k3_df.append(new_row, ignore_index=True)
            print('k3 fitness', k3_fitness)
            
            receiver.nextPacket()
            
        elif '$' == message[0]:
            message = int(message[1:])
            receiver.nextPacket()
            print('removing object')
            obj_node = robot.getFromId(message)
            print(obj_node)
            if obj_node is not None: 
                obj_node.remove()  
            
            
    if '!' not in fitness_scores: 
        # receiver.nextPacket()
        print('will update gene pool --')
        fit_update = True 
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
    global updated 
    
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
        
    run_seconds(1) # runs generation for that given amount of time  
    print('new generation beginning')
    
    
    for gen in range(num_generations-1): 
        
        # pop_fitness = [] 
        
        # send relevant genotypes to each robot, handler
        updated = False 
        
        index = 0 
        for i in range(len(population)):
            emitter.send(str("#"+ str(index) + str(pop_genotypes[index])).encode('utf-8'))
            index +=1 
            
        run_seconds(1) 
        
        print('waiting for genotypes')
        
        run_seconds(1, True) # is waiting until got genotypes
        
        print('found genotypes')
        
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
    
    save_progress()
    # translation_field.setSFVec3f([0,0,0]) # reset robot position
    # rotation_field.setSFRotation([0, 0, 1, 0])
    # khepera_node.resetPhysics()
   

    

         
main()
                    
            
            