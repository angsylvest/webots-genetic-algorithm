from controller import Supervisor, Node, Keyboard, Emitter, Receiver
import statistics 
import pandas as pd

"""
Main supervisor base 
Optimization algorithm - Collaboration-oriented 
Angel Sylvester 2022
"""

# sets up csv for reference 
k_gen_df = pd.DataFrame(columns = ['time step', 'fitness'])

overall_df = pd.DataFrame(columns = ['time', 'objects retrieved'])

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

num_generations = 15
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
simulation_time = 10

global count 
count = 0

def initialize_genotypes():
    global initial_genotypes
    with open('initial_genotype.txt') as f:
        for line in f: 
            initial_genotypes.append(line)

def restore_positions():
    pass 
    
def save_progress():
    # way to save total number of blocks found 
    global k_gen_df
    global overall_df
    
    new_row = {'time': simulation_time*num_generations, 'objects retrieved': total_found}
    overall_df = overall_df.append(new_row, ignore_index=True)
    k_gen_df.to_csv('k_gen_results.csv')
    overall_df.to_csv('overall_results.csv')
    
    print('progress saved to csv')

def message_listener(time_step):
    global k1_df 
    global k2_df 
    global k3_df
    global count 
    global k_gen_df
    global total_found

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
            
        elif 'k-fitness' in message:
            print('message', message)
            k3_fitness = int(message[9:])
            fitness_scores[count] = k3_fitness
            count += 1
            
            new_row = {'time step': time_step, 'fitness': k3_fitness}
            k_gen_df = k_gen_df.append(new_row, ignore_index=True)
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
        
    run_seconds(simulation_time) # runs generation for that given amount of time  
    print('new generation beginning')
    run_seconds(5, True) # is waiting until got genotypes
    
    
    for gen in range(num_generations-1): 
        
        # pop_fitness = [] 
        
        # send relevant genotypes to each robot, handler
        updated = False 
        
        # index = 0 
        # for i in range(len(population)):
            # emitter.send(str("#"+ str(index) + str(pop_genotypes[index])).encode('utf-8'))
            # index +=1 
            
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
                    
            
            