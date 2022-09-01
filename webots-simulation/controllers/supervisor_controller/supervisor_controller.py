from controller import Supervisor, Node, Keyboard, Emitter, Receiver, Field
import statistics 
import pandas as pd

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
initial_genotypes = []

global k1_fitness
global k2_fitness
global k3_fitness
global fitness_scores
fitness_scores = ["!","!","!"]
global pop_genotypes 
pop_genotypes = []

global total_collected 
total_collected = 0 

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
simulation_time = 2

global count 
count = 0

global trials 
trials = 1

global found_list 
found_list = []

# r1 = robot.getFromDef("r1")
# r2 = robot.getFromDef("r2")
# r3 = robot.getFromDef("r3")
# r4 = robot.getFromDef("r4")
# r5 = robot.getFromDef("r5")
# r6 = robot.getFromDef("r6")
# r7 = robot.getFromDef("r7")
# r8 = robot.getFromDef("r8")
# r9 = robot.getFromDef("r9")
# r10 = robot.getFromDef("r10")
# r11 = robot.getFromDef("r11")

arena_area = robot.getFromDef("arena")

def regenerate_environment(block_dist):
    # creates a equally distributed set of blocks 
    # avoiding areas where a robot is already present 
    
    floor_size = arena_area.getField('floorSize')
    print('arena size --', floor_size.getSFVec2f()) 
    tile_size = arena_area.getField('floorTileSize')
    print('tile size --', tile_size.getSFVec2f()) 
    
    num_blocks = 0
    
    starting_pos = [1, 1, 0]
    
    # generate a basic block and place somewhere fanom 
    rootNode = robot.getRoot()
    rootChildrenField = rootNode.getField('children')
    rootChildrenField.importMFNode(-1, 'cylinder-obj.wbo') 
    rec_node = rootChildrenField.getMFNode(-1)
    
    t_field = rec_node.getField('translation')
    t_field.setSFVec3f([0.8,0.78, 0.02]) 

    pass 
    

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
        
    robot.simulationReset()   
    
    coordinates = [[-0.115, 0, 0.0045], [0.2445, 0, 0.0045], [0.6045, 0, 0.0045]]
    for r in range(len(population)): 
        population[r].restartController()
        r_field = population[r].getField('translation')
        r_field.setSFVec3f(coordinates[r])
        
    # tf1 = r1.getField('translation')
    # rf1 = r1.getField('rotation')
    # tf1.setSFVec3f([0.56, 0.29, 0.019])
    # rf1.setSFRotation([0, 1, 0])
    
    # tf2 = r2.getField('translation')
    # rf2 = r2.getField('rotation')
    # tf2.setSFVec3f([0.05, -0.23, 0.019])
    # rf2.setSFRotation([0, 1, 0])
    
    # tf3 = r3.getField('translation')
    # rf3 = r3.getField('rotation')
    # tf3.setSFVec3f([0.13, 0.3, 0.019])
    # rf3.setSFRotation([0, 1, 0])
       
    # tf4 = r4.getField('translation')
    # rf4 = r4.getField('rotation')
    # tf4.setSFVec3f([-0.18, -0.41, 0.019])
    # rf4.setSFRotation([0, 1, 0])
    
    # tf5 = r5.getField('translation')
    # rf5 = r5.getField('rotation')
    # tf5.setSFVec3f([0.31, -0.23, 0.019])
    # rf5.setSFRotation([0, 1, 0])
    
    # tf6 = r6.getField('translation')
    # rf6 = r6.getField('rotation')
    # tf6.setSFVec3f([0.56, -0.23, 0.019])
    # rf6.setSFRotation([0, 1, 0])
    
    # tf7 = r7.getField('translation')
    # rf7 = r7.getField('rotation')
    # tf7.setSFVec3f([0.8, -0.41, 0.019])
    # rf7.setSFRotation([0, 1, 0])
    
    # tf8 = r8.getField('translation')
    # rf8 = r8.getField('rotation')
    # tf8.setSFVec3f([0.37, 0.3, 0.019])
    # rf8.setSFRotation([0, 1, 0])
    
    # tf9 = r9.getField('translation')
    # rf9 = r9.getField('rotation')
    # tf9.setSFVec3f([-0.14, 0.3, 0.019])
    # rf9.setSFRotation([0, 1, 0])
    
    # tf10 = r10.getField('translation')
    # rf10 = r10.getField('rotation')
    # tf10.setSFVec3f([-0.39, 0.57, 0.019])
    # rf10.setSFRotation([0, 1, 0])
    
    # tf11 = r11.getField('translation')
    # rf11 = r11.getField('rotation')
    # tf11.setSFVec3f([0.68, 0.57, 0.019])
    # rf11.setSFRotation([0, 1, 0])
    
    # r2.loadState('init')
    # r3.loadState('init')
    # r4.loadState('init')
    # r5.loadState('init')
    # r6.loadState('init')
    # r7.loadState('init')
    # r8.loadState('init')
    # r9.loadState('init')
    # r10.loadState('init')
    # r11.loadState('init')
        
    # print('end of trial')
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

def message_listener(time_step):
    global k1_df 
    global k2_df 
    global k3_df
    global count 
    global k_gen_df
    global total_found
    global found_list

    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
        
        print('incoming messages', message) 
        
        if message[0] == "$": # handles deletion of objects when grabbed
            # collected_count[int(message[1])] = collected_count[int(message[1])] + 1
            print('removing object')
            message = message[1:]
            print(message)
            obj_node = robot.getFromId(int(message))
            print(obj_node)
            if obj_node is not None:
                t_field = obj_node.getField('translation')
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
                         
            if total_found == 11:
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
        
    run_seconds(simulation_time) # runs generation for that given amount of time  
    print('new generation beginning')
    run_seconds(5, True) # is waiting until got genotypes
    
    
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
    restore_positions()
    initialize_genotypes()
    run_optimization()
    save_progress()
    # translation_field.setSFVec3f([0,0,0]) # reset robot position
    # rotation_field.setSFRotation([0, 0, 1, 0])
    # khepera_node.resetPhysics()
  
         
main()
                    
            
            