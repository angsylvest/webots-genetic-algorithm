from controller import Supervisor, Node, Keyboard, Emitter, Receiver, Field
import math 
from robot_pop import * 

"""
Main supervisor base 
Optimization algorithm - Collaboration-oriented 
Angel Sylvester 2022
"""
columns = 'agent id' + ',time step' + ',fitness' + ',xpos'+ ',ypos' + ',num col' + ',genotype' 

# global collected_count 
collected_count = []

# collected counts csv generation 
overall_f = open('../../graph-generation/collection-data/overall-df.csv', 'w')
overall_columns = 'trial' + ',time' + ',objects retrieved' + ',size' + ',type'
overall_f.write(str(overall_columns) + '\n')
overall_f.close()
overall_f = open('../../graph-generation/collection-data/overall-df.csv', 'a')

# for individual robot, statistics about strategy taken over time & individual collision info 
strategy_f = open("../../graph-generation/collision-data/ga-info.csv", 'w')
strategy_f.write('agent id'+ ',time step' + ',straight' + ',alternating-left' + ',alternating-right' + ',true random' + ',time since last block' + ',size' + ',collisions'+ ',size'+ ',type' + '\n')
strategy_f.close()

# genetic algorithm-specific parameters 
num_generations = 10
simulation_time = 30
trials = 30
robot_population_sizes = [5, 10, 15]
gene_list = ['control speed 10', 'detection threshold 1000', 'follower threshold 5']
curr_size = 5

# statistics collected 
population = []
initial_genotypes = []
pop_genotypes = [] 
found_list = []
total_found = 0
block_list = []
reproduce_list = []
r_pos_to_generate = []
pairs = []
fitness_scores = []

# set-up robot 
TIME_STEP = 32
robot = Supervisor()  # create Supervisor instance
emitter = robot.getDevice("emitter")
emitter.setChannel(1)
receiver = robot.getDevice("receiver") 
receiver.enable(TIME_STEP)
receiver.setChannel(2) 
taken = False # getting second child assigned 
updated = False
fit_update = False 


# set up environments 
def generate_robot_central(num_robots):
    global fitness_scores 
    global collected_count 
    global population
    global columns 
    global r_pos_to_generate
    global pairs 
    
    initialize_genotypes(num_robots)
    emitter.send(str("size-" + str(num_robots)).encode('utf-8'))
    
    if len(population) != 0: 
    
        for r in population: 
            r.remove()
             
    population = []
    fitness_scores = []
    collected_count = []
    pairs = []
        
    for i in range(num_robots):
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../las_supervisor/robots/robot-updated-ga.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        pose = [round(random.uniform(0.25, -0.25),2), round(random.uniform(0.25, -0.25) ,2), 0.02]
        while pose in r_pos_to_generate: # remove any duplicates
            pose = [round(random.uniform(0.25, -0.25),2), round(random.uniform(0.25, -0.25) ,2), 0.02]
        r_pos_to_generate.append(pose)
        t_field.setSFVec3f(pose)
                # print(r_field)
        
        # sets up metrics 
        fitness_scores.append("!")
        pairs.append("!")
        collected_count.append(0)
        population.append(rec_node)
        
    
def regenerate_blocks_random(): # essentially dual source 

    global block_list
    for obj in block_list: 
        obj.remove()
    
    block_list = []
    
    # generates block on opposite sides of arena (randomly generated) 
    for i in range(10): 
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        t_field.setSFVec3f([round(random.uniform(0.9, -0.9),2), round(random.uniform(0.3, 0.85),2), 0.02]) 
        block_list.append(rec_node)
    
    for i in range(10): 
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        t_field.setSFVec3f([round(random.uniform(0.9, -0.9),2), round(random.uniform(-1, 0.23),2), 0.02]) 
        block_list.append(rec_node)
        
    
def generate_blocks_single_source():

    for i in range(40): 
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
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
    
    # generates block on opposite sides of arena (randomly generated) 
    for i in range(10): 
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        t_field.setSFVec3f([round(random.uniform(0.9, -0.9),2), round(random.uniform(0.3, 0.85),2), 0.02]) 
        block_list.append(rec_node)
    
    for i in range(10): 
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
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
    for r in range(size):
        new_geno = create_individal_genotype(gene_list)
        initial_genotypes.append(new_geno)
        pop_genotypes.append(new_geno)
     
    
def find_nearest_robot_genotype(r_index):
    global population 
    global reproduce_list 
    global collected_count 
    global pairs 
    
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
    # use emitter to send genotype to corresponding robot if fitness is better and if nearby 
    if type(curr_fitness) == 'int' and type (other_fitness) == 'int' and (other_fitness < (curr_fitness + 2) or collected_count[r_index] < collected_count[other_index]) : 
        emitter.send('potential partner-' + str(pop_genotypes[other_index]) + '-' + str(other_fitness) + '-' + str(collected_count[other_index]))
        pairs.append(r_index) # this robot now has a potential partner 
                
    
def save_progress():
    # way to save total number of blocks found 
    global overall_df
    overall_f.close()
    print('progress saved to csv')
    emitter.send('sim-complete'.encode('utf-8'))

def message_listener(time_step):
    global total_found 
    global collected_count 
    global found_list
    global pop_genotypes
    global reproduce_list 
    global population
    global curr_size

    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
        
        if message[0] == "$": # handles deletion of objects when grabbed
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
            index = int(message.split('-')[0][1:])
            partner = message.split('-')[2][5:]
            fitness_scores[int(index)] = fit
            
            print('fitness' , message, index, fit)
            
            print('updated with encountered partner') 
            if partner != 'none':
                new_geno = reproduce(pop_genotypes[index], pop_genotypes[partner])
                pop_genotypes[index] = new_geno
            
            receiver.nextPacket()
            
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
            else: 
                break 
        
        elif robot.getTime() - start > new_t: 
            emitter.send('return_fitness'.encode('utf-8'))
            print('requesting fitness')
            break 

        elif not waiting: 
            # constantly checking for messages from robots 
            message_listener(robot.getTime())
                         
            if total_found == len(block_list):
                emitter.send('return_fitness'.encode('utf-8'))
                print('requesting fitness')
                print('collected all objects')
                break      
    return 
   
# will use selected partners from each robot and reproduce with that corresponding index, and update population at the end of gen          
def update_geno_list(genotype_list): 
    global fitness_scores
    global pop_genotypes 
    global gene_list
    global taken 
    global updated 
    global population 
    global pairs 
    
    # only makes executive changes if it's better off to just re-randomize population   
    if max(fitness_scores) == 0:
        cp_genotypes = pop_genotypes.copy()
        for i in range(len(population)):
            if i not in pairs: 
                g = create_individal_genotype(gene_list)
                new_offspring = reproduce(cp_genotypes[i], g)
                # print('updated genolist --', g)
                pop_genotypes[i] = new_offspring
                     
    # update parameters to hopefully improve performance
    print('curr population --', population, len(population))
    
    # fitness_scores = []
    fitness_scores = ["!" for i in range(len(population))]
    pairs = ["!" for i in range(len(population))]
    fit_update = False 
    print('gene pool updated') 
    updated = True

# fitness function for each individual robot 
def eval_fitness(time_step):
    global pop_genotypes 
    global fitness_scores 
    global fit_update
    global population 
            
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
    global population 
    
    # initialize genotypes 
    generate_robot_central(5)
    reset_genotype()
    regenerate_environment(0.2)
    
    # fixes robots falling over
    for rec_node in population: 
        r_field = rec_node.getField('rotation')
        if r_field.getSFRotation() != [0, 0, -1]:
            r_field.setSFRotation([0, 0, -1])
            
    run_seconds(simulation_time) # runs generation for that given amount of time  
    print('new generation beginning')
    run_seconds(5, True) # is waiting until got genotypes
    
    for size in robot_population_sizes:
        curr_size = size  
        initialize_genotypes(size)
        r_pos_to_generate = []
        generate_robot_central(size)
        
        for i in range(trials): 
            print('beginning new trial', i)
            for gen in range(num_generations-1): 
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

            overall_f.write(str(i) + ',' + str(robot.getTime()) + ',' + str(total_found) + ',' + str(size)+ ',' + 'ga' + '\n')    
            overall_f.close()
            overall_f = open('overall-df.csv', 'a')
            print('items collected', total_found)
            regenerate_environment(0.2)  
            total_found = 0 
            reproduce_list = []
            found_list = []
            reset_genotype()              
    return 
  
def main(): 
    run_optimization()
    save_progress()
         
main()
                    
            
            
