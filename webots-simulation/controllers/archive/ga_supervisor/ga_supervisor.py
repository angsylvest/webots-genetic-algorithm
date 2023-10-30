from controller import Supervisor, Node, Keyboard, Emitter, Receiver, Field
import math 
from robot_pop import * 

"""
Main supervisor base 
Optimization algorithm - Collaboration-oriented 
Angel Sylvester 2022
"""

columns = 'agent id' + ',time step' + ',fitness' + ',xpos'+ ',ypos' + ',num col' + ',genotype' 

# collected counts csv generation 
overall_f = open('../../graph-generation/collection-data/overall-df.csv', 'w')
overall_columns = ['trial','time', 'objects retrieved', 'size']
overall_f.write(str(overall_columns) + '\n')
# overall_df = pd.DataFrame(columns = ['trial','time', 'objects retrieved'])
overall_f.close()
overall_f = open('overall-df.csv', 'a')

# for individual robot, statistics about strategy taken over time 
strategy_f = open("../generalized_ga_controller/ga-info.csv", 'w')
strategy_f.write('agent id'+ ',time step' + ',straight' + ',alternating-left' + ',alternating-right' + ',true random' + ',time since last block' + ',size' + ',collisions'+ ',size' + '\n')
strategy_f.close()

# genetic algorithm-specific parameters 
num_generations = 10
simulation_time = 10
trials = 30
robot_population_sizes = [15]
curr_size = robot_population_sizes[0] 
gene_list = ['control speed 10', 'detection threshold 1000', 'time switch 550']

# statistics collected 
collected_count = []
initial_genotypes = []
fitness_scores = []
pop_genotypes = [] 
found_list = []
total_found = 0
population = []

# simulation specific setup 
TIME_STEP = 32
robot = Supervisor()  # create Supervisor instance

global curr_df

# set up messenging system 
emitter = robot.getDevice("emitter")
emitter.setChannel(1)
receiver = robot.getDevice("receiver") 
receiver.enable(TIME_STEP)
receiver.setChannel(2) 

# sim specific parameters 
taken = False # getting second child assigned 
updated = False
fit_update = False 
block_list = []
reproduce_list = []
r_pos_to_generate = []

# generate environment type: random 
def generate_robot_central(num_robots):
    global fitness_scores 
    global collected_count 
    global population
    global columns 
    global curr_df
    global r_pos_to_generate
    
    initialize_genotypes(num_robots)
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
        rootChildrenField.importMFNode(-1, '../supervisor_controller/robots/robot-ga.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        pose = [round(random.uniform(0.25, -0.25),2), round(random.uniform(0.25, -0.25) ,2), 0.02]
        r_pos_to_generate.append(pose)
        t_field.setSFVec3f(pose)
        
        # sets up metrics 
        fitness_scores.append("!")
        collected_count.append(0)
        population.append(rec_node)
        
        curr_df = open('robot-info-' + str(num_robots) + '.csv', 'a')
        # k2_f = open('robot-2-info.csv', 'a')
        # k3_f = open('robot-3-info.csv', 'a') 
        
    pass 
    
def regenerate_blocks_random(): # essentially dual source 

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
        
        
def regenerate_environment(block_dist):
    # creates a equally distributed set of blocks 
    # avoiding areas where a robot is already present 
    global block_list
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
        

def initialize_genotypes(size):
    global initial_genotypes
    global gene_list
    global pop_genotypes 
    # initial_geno_txt = open('initial_genotype.txt', 'w')
    
    lines = []
    pop_genotypes = []
    initial_genotypes = []

    for r in range(size):
        new_geno = create_individal_genotype(gene_list)
        initial_genotypes.append(new_geno)
        pop_genotypes.append(new_geno)
        # emitter.send(str("#"+ str(r) + str(genotype)).encode('utf-8'))
    
    # with open('initial_genotype.txt') as f:
        # for line in f: 
            # initial_genotypes.append(line)

# def restore_positions():
        # clearing messages between each trial 
    # while receiver.getQueueLength()>0:
        # message = receiver.getData().decode('utf-8')
        # receiver.nextPacket()
        
    # robot.simulationReset()   
    
    # manually resets arena configuration (will automate soon or save as csv) 
    # coordinates = [[-0.115, 0, 0.0045], [0.2445, 0, 0.0045], [0.6045, 0, 0.0045]]
    # for r in range(len(population)): 
        # population[r].restartController()
        # r_field = population[r].getField('translation')
        # r_field.setSFVec3f(coordinates[r])
        
    # print('end of trial')
    # initialize_genotypes()
     
    
def find_nearest_robot_genotype(r_index):
    global population 
    global reproduce_list 
    closest_neigh = " "
    curr_robot = population[r_index]
    curr_dist = 1000 # arbitrary value 
    curr_fitness = fitness_scores[r_index]
    other_fitness = 0
    
    curr_pos = [curr_robot.getPosition()[0], curr_robot.getPosition()[1]]
    other_index = r_index
    
    for i in range(len(population)):
        if (i != r_index): 
            other_pos = [population[i].getPosition()[0], population[i].getPosition()[0]]
            dis = math.dist(curr_pos, other_pos)
            if closest_neigh == " ":
                closest_neigh = str(population[i].getId())
                curr_dist = dis
                other_fitness = fitness_scores[i]
                other_index = i
            elif dis < curr_dist: 
                closest_neigh = str(population[i].getId())
                curr_dist = dis 
                other_fitness = fitness_scores[i]
                other_index = i
    # print('found closest neighbor', closest_neigh)
    # use emitter to send genotype to corresponding robot if fitness is better and if nearby 
    if type(curr_fitness) == 'int' and type (other_fitness) == 'int' and other_fitness < (curr_fitness + 2): 
        reproduced_geno = reproduce(pop_genotypes[r_index], pop_genotypes[i])
        pop_genotypes[r_index] = new_geno
        reproduce_list.append(r_index)
        emitter.send(str("#"+ str(r_index) + str(reproduced_geno)).encode('utf-8'))
        # will probably increase fitness between these two for communciation
                
    
def save_progress():
    # way to save total number of blocks found 
    global overall_df
    global df_list 
    # global k1_df
    # global k2_df 
    # global k3_df
    
    # global overall_f
    # global df_list 
    # global k1_f
    # global k2_f 
    # global k3_f  
    
    global curr_df
    
    # k1_f.close()
    # k2_f.close() 
    # k3_f.close()    
    curr_df.close() 
    overall_f.close()
 
    # generate_fitness_csvs(df_list)
    # generate_fitness("summary-fitness.csv")
    # overall_df.to_csv('overall_results.csv')
    
    print('progress saved to csv')
    emitter.send('sim-complete'.encode('utf-8'))

def message_listener(time_step):
    global k1_f 
    global k2_f 
    global k3_f
    global total_found 
    # global df_list 
    global collected_count 
    global found_list
    global pop_genotypes
    global reproduce_list 
    global population
    global curr_df
    global curr_size

    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
        
        # print('incoming messages', message) 
        
        if message[0] == "$": # handles deletion of objects when grabbed
            # collected_count[int(message[1])] = collected_count[int(message[1])] + 1
            # print('removing object')
            # message = message[2:]
            # print(message)
            # print(obj_node)
            obj_node = robot.getFromId(int(message.split("-")[1]))
            
            if obj_node is not None:
                r_node_loc = population[int(message.split("-")[0][1:])].getField('translation').getSFVec3f()
                t_field = obj_node.getField('translation')
                t_node_loc = t_field.getSFVec3f()
                
                if (math.dist(r_node_loc, t_node_loc) < 0.15):
                    t_field.setSFVec3f([-0.9199,-0.92, 0.059]) 
                    # obj_node.remove()
                    # remove redundant requests 
                    if obj_node not in found_list:
                        total_found += 1
                        found_list.append(obj_node)
                        collected_count[int(message.split("-")[0][1:])] = collected_count[int(message.split("-")[0][1:])] + 1
                        msg = "%" + message[1:]
                        emitter.send(str(msg).encode('utf-8'))
                    
            receiver.nextPacket()
        
        elif 'fitness' in message: 
            fit = message.split('-')[1][7:] 
            index = message.split('-')[0][1:]
            fitness_scores[int(index)] = fit
            print('fitness' , message, index, fit)
            
            curr_df.write('agent id,' + str(index) + ',time step, ' + str(robot.getTime()) + ',fitness,' + str(fit) + ',xpos,' + str(population[int(index)].getPosition()[0]) + ',ypos,' + str(population[int(index)].getPosition()[1]) + ',num col,' + str(collected_count[int(index)]) + ',genotype,' + str(pop_genotypes[int(index)])+ '\n')
            curr_df.close()
            curr_df = open('robot-info-' + str(curr_size) + '.csv', 'a')
            receiver.nextPacket()
            pass # will be generalized 
            
               
        # elif 'k1-fitness' in message: 
            # k1_fitness = int(message[10:])
            # fitness_scores[0] = k1_fitness
            
            # k1_f.write('agent id:' + str(1) + ',time step: ' + str(time_step) + ',fitness:' + str(k1_fitness) + ',xpos:' + str(k1.getPosition()[0]) + ',ypos:' + str(k1.getPosition()[1]) + ',num col:' + str(collected_count[0]) + ',genotype:' + str(pop_genotypes[0]))
            
            # new_row = {'agent id': 1, 'time step': time_step, 'fitness': k1_fitness, 'xpos': k1.getPosition()[0], 'ypos': k1.getPosition()[1], 'num col': collected_count[0], 'genotype':pop_genotypes[0]}
            # k1_df = pd.concat([k1_df, pd.DataFrame([new_row])], ignore_index=True)
            # df_list[0] = k1_f
            
            # print('k1 fitness', k1_fitness)
            
            # receiver.nextPacket()
            
        # elif 'k2-fitness' in message: 
            # k2_fitness = int(message[10:])
            # fitness_scores[1] = k2_fitness
            
            # k1_f.write('agent id:' + str(2) + ',time step: ' + str(time_step) + ',fitness:' + str(k2_fitness) + ',xpos:' + str(k2.getPosition()[0]) + ',ypos:' + str(k2.getPosition()[1]) + ',num col:' + str(collected_count[1]) + ',genotype:' + str(pop_genotypes[1])) 
            
            # new_row = {'agent id': 2,'time step': time_step, 'fitness': k2_fitness, 'xpos': k2.getPosition()[0], 'ypos': k2.getPosition()[1], 'num col': collected_count[1], 'genotype':pop_genotypes[1]}
            # k2_df = pd.concat([k2_df, pd.DataFrame([new_row])], ignore_index = True)
            # df_list[1] = k2_f
            
            # print('k2 fitness', k2_fitness)
            
            # receiver.nextPacket()
            
        # elif 'k3-fitness' in message:
            # k3_fitness = int(message[10:])
            # fitness_scores[2] = k3_fitness
            
            # k1_f.write('agent id:' + str(2) + ',time step: ' + str(time_step) + ',fitness:' + str(k3_fitness) + ',xpos:' + str(k3.getPosition()[0]) + ',ypos:' + str(k3.getPosition()[1]) + ',num col:' + str(collected_count[2]) + ',genotype:' + str(pop_genotypes[2]))
            
            # new_row = {'agent id': 3,'time step': time_step, 'fitness': k3_fitness, 'xpos': k3.getPosition()[0], 'ypos': k3.getPosition()[1], 'num col': collected_count[2], 'genotype':pop_genotypes[2]}
            # k3_df = pd.concat([k3_df, pd.DataFrame([new_row])], ignore_index = True)
            # df_list[2] = k3_f
            
            # print('k3 fitness', k3_fitness)
            
            # receiver.nextPacket()
            
        elif 'encounter' in message: 
            print('robot found -- checking genotype') 
            robo_index = int(message.split('-')[0])
            # reproduce_list.append(robo_index) 
            new_geno = find_nearest_robot_genotype(robo_index)
            
            receiver.nextPacket()
            
        else: 
            
            receiver.nextPacket()
        
            
    
    
# runs simulation for designated amount of time 
def run_seconds(t,waiting=False):
    global pop_genotypes
    global fitness_scores
    global updated
    global fit_update 
    global block_list
    
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
                print('requesting fitness')
                print('collected all objects')
                break      
    return 
            
def update_geno_list(genotype_list): 
    global fitness_scores
    global pop_genotypes 
    global gene_list
    global taken 
    global updated 
    global population 
    
        
    if max(fitness_scores) == 0:
        # update all genotypes 
        pop_genotypes = []
        
        for i in range(len(population)):
            g = create_individal_genotype(gene_list)
            # print('updated genolist --', g)
            pop_genotypes[i] = g
        
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
        
        for g in range(len(cp_genotypes)): 
            if g not in reproduce_list or g != max_index: 
                child = reproduce(cp_genotypes[g], pop_genotypes[max_index])
                cp_genotypes[g] = child
        
        # cp_genotypes.remove(pop_genotypes[max_index])
        # child = reproduce(cp_genotypes[0], pop_genotypes[max_index])
        # other_child = reproduce(pop_genotypes[max_index], cp_genotypes[1])
               
        # for i in range(len(fitness_scores)):
            # if i != max_index and not taken: 
                # pop_genotypes[0] = child
                # taken = True 
            # elif i != max_index and taken: 
                # pop_genotypes[1] = other_child
                # taken = False 
        
        # replace genotypes of one of parents 
                     
    # update parameters to hopefully improve performance
    print('curr population --', population, len(population))
    
    # fitness_scores = []
    fitness_scores = ["!" for i in range(len(population))]
    fit_update = False 
    print('gene pool updated') 
    updated = True
    
 
    

# fitness function for each individual robot 
def eval_fitness(time_step):
    global pop_genotypes 
    global fitness_scores 
    global fit_update
    # global k1_df 
    # global k2_df 
    # global k3_df
    global population 
    
    # print('fitness scores ', fitness_scores)
            
    if '!' not in fitness_scores: 
        # receiver.nextPacket()
        print('will update gene pool --')
        fit_update = True 
        update_geno_list(pop_genotypes)

def reset_genotype():
    index = 0 
    global population 
    global pop_genotypes 
    pop_genotypes = []
    
    for i in range(len(population)):
        # genotype = create_individal_genotype(gene_list)
        # print('genotype', i, genotype)
        genotype = initial_genotypes[i]
        pop_genotypes.append(genotype)
        emitter.send(str("#"+ str(index) + str(genotype)).encode('utf-8'))
        index +=1   
    
def run_optimization():
    global pop_genotypes 
    global gene_list 
    global updated
    global simulation_time 
    global overall_f
    global total_found 
    global collected_count
    global found_list
    global reproduce_list 
    global r_pos_to_generate
    global curr_size
    
    # initialize genotypes 
    # will be same genotype as normal (for comparison purposes) 
   
        
    generate_robot_central(robot_population_sizes[0])
    reset_genotype()
    regenerate_environment(0.2) 
    run_seconds(simulation_time) # runs generation for that given amount of time  
    print('new generation beginning')
    run_seconds(5, True) # is waiting until got genotypes
    
    for size in robot_population_sizes:
    
        curr_size = size  
        
        initialize_genotypes(size)
                # creates a csv specific to the robot 
        curr_df = open('robot-info-' + str(size) + '.csv', 'w')
        # k2_f = open('robot-2-info.csv', 'w')
        # k3_f = open('robot-3-info.csv', 'w')
        
        curr_df.write(str(columns)+ '\n')
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
                
                # pop_fitness = [] 
                
                # send relevant genotypes to each robot, handler
                updated = False 
                
                index = 0 
                
                print('number in population', len(population))
                print('number of genotypes',  len(pop_genotypes), 'for size: ', size)
                
                
                for i in range(len(population)):
                    emitter.send(str("#"+ str(index) + str(pop_genotypes[index])).encode('utf-8'))
                    index +=1 
                    
                run_seconds(simulation_time) 
                
                print('waiting for genotypes')
                
                run_seconds(5, True) # is waiting until got genotypes
                
                print('found genotypes')
                print('new generation starting -')
                reproduce_list = []
                # print('trial --' ,i)
            
            overall_f.write('trial,' + str(i) + ',time,' + str(robot.getTime()) + ',objects retrieved,' + str(total_found) + ',size,' + str(size)+ '\n')    
            overall_f.close()
            overall_f = open('overall-df.csv', 'a')
            # new_row = {'trial': i,'time': simulation_time*num_generations, 'objects retrieved': total_found}
            print('items collected', total_found)
            # overall_df = pd.concat([overall_df, pd.DataFrame([new_row])], ignore_index = True)
            # restore_positions() 
            # generate_robot_central(size)
            regenerate_environment(0.2)  
            total_found = 0 
            reproduce_list = []
            # collected_count = [0, 0, 0]
            found_list = []
            reset_genotype()    
        curr_df.close()           
    return 
    
   
   
        
            
def main(): 
    run_optimization()
    save_progress()
         
main()
                    
            
            
