from controller import Supervisor, Node, Keyboard, Emitter, Receiver, Field
import math 
from robot_pop import * 
import random
import ast

# ensure that we can access utils package to streamline tasks 
import sys 
sys.path.append('../../')
import utils.environment as env_mod 
import utils.globals as globals
import utils.k_means as k_means
import utils.shared_map as shared_map

from math import pi

"""
Main supervisor base 
Optimization algorithm - Collaboration-oriented 
Angel Sylvester 2022
"""

columns = 'agent id' + ',time step' + ',fitness' + ',xpos'+ ',ypos' + ',num col' + ',genotype' + ',potential time'

# global collected_count 
collected_count = []

# genetic algorithm-specific parameters 
num_generations = 10
simulation_time = 30
trials = 25
curr_trial = 0 
robot_population_sizes = [5, 10, 15]
gene_list = ['control speed 10', 'energy cost 5', 'food energy 30', 'observations thres 5']
curr_size = robot_population_sizes[0]
env_type = globals.env_type # "single source" # "power law"
sim_type = globals.sim_type #"random"
communication = globals.communication # True
high_dense = globals.using_high_dens #True
using_bayes = globals.using_bayes

assigned_teams = {}

# collected counts csv generation 
overall_f = open(f'../../graph-generation/collection-data/overall-df-{sim_type}-{curr_size}-comm_{communication}-dense_{high_dense}.csv', 'w')
overall_columns = 'trial' + ',time' + ',objects retrieved' + ',size' + ',type' + ',potential time' + ',total elapsed'
overall_f.write(str(overall_columns) + '\n')
overall_f.close()
overall_f = open(f'../../graph-generation/collection-data/overall-df-{sim_type}-{curr_size}-comm_{communication}-dense_{high_dense}.csv', 'a')

# for individual robot, statistics about strategy taken over time & individual collision info 
strategy_f = open(f"../../graph-generation/collision-data/ga-info-{sim_type}-{curr_size}-comm_{communication}-dense_{high_dense}.csv", 'w')
strategy_f.write('agent id'+ ',time step' + ',straight' + ',alternating-left' + ',alternating-right' + ',true random' + ',time since last block' + ',num encounters' + ',size' + ',fitness'+ ',size'+ ',type' + ',trial' + ',collected' + ',genotype' + ',num better' + ',pos x' + ',pos y' + '\n')
strategy_f.close()

gene_df = open(f"../../graph-generation/collision-data/ga-gene-info-{sim_type}-{curr_size}-comm_{communication}-dense_{high_dense}.csv", 'w')
gene_df.write('agent id'+ ',time step' + ',trial' + ',size' + ',genotype' + '\n')
gene_df.close()


# statistics collected 
population = []
initial_genotypes = []
pop_genotypes = [] 
found_list = []
total_found = 0
block_list = []
reproduce_list = []
r_pos_to_generate = []
b_pos_to_generate = []
b_pos_to_generate_alternative = []
pairs = []
fitness_scores = []
overall_fitness_scores = []

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
start = 0 

prev_msg = ""
seed_val = 11
random.seed(seed_val)
id_msg = ""
id_ind = ""

emitter_individual = robot.getDevice("emitter_processor")
emitter_individual.setChannel(5)
assessing = False 
repopulate = False # keep False for now 
phase_one_times = [620]
central = True

# generate envs 
curr_env = env_mod.Environment(env_type=env_type, seed = seed_val)

input_from_others = {}
info_garnered = []
prev_time = robot.getTime()
msg = ""

def message_sender(msg, individual = False):
    if not individual: 
        emitter.send(msg.encode('utf-8'))
    else:  
        emitter_individual.send(msg.encode('utf-8'))

# set up environments 
def generate_robot_central(num_robots):
    global fitness_scores 
    global collected_count 
    global population
    global columns 
    global r_pos_to_generate
    global pairs 
    global overall_fitness_scores
    global prev_msg 
    global id_msg
    global id_ind
    
    global input_from_others
    
    initialize_genotypes(num_robots)
    curr_msg = str("size-" + str(num_robots))
    if curr_msg != prev_msg: 
        emitter.send(str("size-" + str(num_robots)).encode('utf-8'))
        emitter_individual.send(str("size-" + str(num_robots)).encode('utf-8'))
        prev_msg = curr_msg
    
    if len(population) != 0: 
    
        for r in population: 
            r.remove()
             
    population = []
    fitness_scores = []
    overall_fitness_scores = []
    collected_count = []
    pairs = []
    id_msg = "ids"
    id_ind = "id_ind"
        
    for i in range(num_robots):
        curr_key = f'agent-{i}'
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../las_supervisor/robots/robot-ga-update.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
        input_from_others[curr_key] = 0
        
    
        t_field = rec_node.getField('translation')
        pose = [round(random.uniform(0.3, -0.3),2), round(random.uniform(0.3, -0.3) ,2), 0.02]
        while pose in r_pos_to_generate: # remove any duplicates
            pose = [round(random.uniform(0.3, -0.3),2), round(random.uniform(0.3, -0.3) ,2), 0.02]
        r_pos_to_generate.append(pose)
        t_field.setSFVec3f(pose)
                # print(r_field)
        
        # sets up metrics 
        fitness_scores.append("!")
        overall_fitness_scores.append('!')
        pairs.append("!")
        collected_count.append(0)
        population.append(rec_node)
        id_msg += " " + str(rec_node.getId()) 
        id_ind += " " + curr_key

# set up environments 
def generate_robot_edge(num_robots, right = False):
    global fitness_scores 
    global collected_count 
    global population
    global columns 
    global r_pos_to_generate
    global pairs 
    global overall_fitness_scores
    global prev_msg 
    global id_msg
    
    initialize_genotypes(num_robots)
    curr_msg = str("size-" + str(num_robots))
    if curr_msg != prev_msg: 
        emitter.send(str("size-" + str(num_robots)).encode('utf-8'))
        emitter_individual.send(str("size-" + str(num_robots)).encode('utf-8'))
        prev_msg = curr_msg
    
    if len(population) != 0: 
    
        for r in population: 
            r.remove()
             
    population = []
    fitness_scores = []
    overall_fitness_scores = []
    collected_count = []
    pairs = []
    id_msg = "ids"
        
    for i in range(num_robots):
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../las_supervisor/robots/robot-ga-update.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
        
        if right: 
            print('right')
            t_field = rec_node.getField('translation')
            pose = [round(random.uniform(-0.5, -0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02]
            while pose in r_pos_to_generate: # remove any duplicates
                pose = [round(random.uniform(-0.5, -0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02]
            r_pos_to_generate.append(pose)
            t_field.setSFVec3f(pose)
                # print(r_field)

        else: 
            print('left')
            t_field = rec_node.getField('translation')
            pose = [round(random.uniform(0.5, 0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02]
            while pose in r_pos_to_generate: # remove any duplicates
                pose = [round(random.uniform(0.5, 0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02]
            r_pos_to_generate.append(pose)
            t_field.setSFVec3f(pose)
                # print(r_field)            
        
        # sets up metrics 
        fitness_scores.append("!")
        overall_fitness_scores.append('!')
        pairs.append("!")
        collected_count.append(0)
        population.append(rec_node)
        id_msg += " " + str(rec_node.getId()) 


def regenerate_blocks(seed = None):
    global block_list
    global population 
    global r_pos_to_generate
    global b_pos_to_generate
    global curr_env

    global population 
    
    for obj in block_list: 
        obj.remove()
    
    block_list = []
    assert curr_env

    for i in range(len(r_pos_to_generate)):
        population[i].getField('translation').setSFVec3f(r_pos_to_generate[i])
    
    if seed == 15 and curr_env.seed != 15: 
        curr_env = env_mod.Environment(env_type=env_type, seed = seed)
        b_pos_to_generate = curr_env.generate_blocks()

    if len(b_pos_to_generate) == 0:
        b_pos_to_generate = curr_env.generate_blocks()

    for i in b_pos_to_generate: 
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        t_field.setSFVec3f(i) 
        
        r_field = rec_node.getField('rotation')
        if r_field.getSFRotation() != [0, 0, -1]:
            r_field.setSFRotation([0, 0, -1])
            
        block_list.append(rec_node)
            
    for rec_node in block_list: # set to be upright
        r_field = rec_node.getField('rotation')
        if r_field.getSFRotation() != [0, 0, -1]:
            r_field.setSFRotation([0, 0, -1])
            
# calculates angle normal to current orientation 
def calc_normal(curr_angle): 

    if (curr_angle + round(pi/2, 2) <= round(pi, 2) and curr_angle <= round(pi, 2) and curr_angle >= 0): 
        return round(curr_angle + round(pi/2, 2), 2)
    
    elif (curr_angle + round(pi/2, 2) > round(pi, 2) and curr_angle < round(pi, 2) and curr_angle > 0): 
        diff = round(pi/2, 2) - (round(pi,2) - curr_angle) 
        return round((-1*round(pi/2, 2) + diff),2)
    
    elif (curr_angle + round(pi/2, 2) < 0 and curr_angle < 0): 
        return round((-1*round(pi, 2) + curr_angle + round(pi/2, 2)),2)
        
    elif (curr_angle + round(pi/2, 2) >= 0 and curr_angle <= 0): 
        diff = abs(round(pi/2, 2) - curr_angle) 
        return round(diff,2) 
        
    elif (curr_angle == round(pi,2)): # handle edge case that seems to only happen w/exactly 3.14 (never broke before because never quite at 3.14????)
        return round(-1*round(pi/2, 2),2)

def initialize_genotypes(size):
    global initial_genotypes
    global gene_list 
    global pop_genotypes 
    # initial_geno_txt = open('initial_genotype.txt', 'w')
    
    lines = []
    pop_genotypes = []
    for r in range(size):
        new_geno = create_individal_genotype(gene_list)
        # print(new_geno)
        initial_genotypes.append(new_geno)
        pop_genotypes.append(new_geno)
                
    
def save_progress():
    # way to save total number of blocks found 
    global overall_df
    overall_f.close()
    print('progress saved to csv')
    emitter.send('sim-complete'.encode('utf-8'))

    for i in range(20): 
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        t_field.setSFVec3f([round(random.uniform(-0.5, -0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02]) 
        block_list.append(rec_node)        
      
    for i in range(20): 
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
        
        t_field = rec_node.getField('translation')
        t_field.setSFVec3f([round(random.uniform(0.5, 0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02]) 
        block_list.append(rec_node)  
            
def message_listener(time_step):
    global total_found 
    global collected_count 
    global found_list
    global pop_genotypes
    global reproduce_list 
    global population
    global curr_size
    global overall_fitness_scores
    global start 
    global simulation_time
    global prev_msg 

    if receiver.getQueueLength()>0 and (robot.getTime() - start < simulation_time):
        message = receiver.getData().decode('utf-8')
        # print('supervisor msgs --', message) 
        
        if message[0] == "$": # handles deletion of objects when grabbed
            obj_node = robot.getFromId(int(message.split("-")[1]))
            
            if obj_node is not None:
                r_node_loc = population[int(message.split("-")[0][1:])].getField('translation').getSFVec3f()
                t_field = obj_node.getField('translation')
                t_node_loc = t_field.getSFVec3f()
                
                if (math.dist(r_node_loc, t_node_loc) < 0.15):
                    if repopulate: 
                        # will be placed somewhere random 
                        side = random.randint(0,1)
                        if side == 1:
                            t_field.setSFVec3f([round(random.uniform(-0.5, -0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02]) 
                        else: 
                            t_field.setSFVec3f([round(random.uniform(0.5, 0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02])   
                    else:
                        t_field.setSFVec3f([-0.9199,-0.92, 0.059]) 
                    # obj_node.remove()
                    # remove redundant requests 
                    if obj_node not in found_list:
                        total_found += 1
                        if not repopulate: 
                            found_list.append(obj_node)
                            
                        collected_count[int(message.split("-")[0][1:])] = collected_count[int(message.split("-")[0][1:])] + 1
                        msg = "%" + message[1:]
                        if prev_msg != msg: 
                            emitter.send(str(msg).encode('utf-8'))
                            prev_msg = msg
                    
            receiver.nextPacket()


        if 'agent' in message:
            print(f'agent message: {message}')
            agent_dict = {}
            info_garnered = []
            message_parsed = message.split('~')[0].split('-')
            # print(f'parsed msg: {message_parsed}')
            agent_info = message_parsed[0].split(':')[1]
            # Convert the string to a dictionary using ast.literal_eval()
            proposed_strat = ast.literal_eval(message_parsed[1].split(':')[1])
            # print(f'parsed strat: {message.split("~")[1][5:]}')
            strat_parsed = ast.literal_eval(str(message.split('~')[1][5:]))
            
            
            poses = ast.literal_eval(message.split('~')[0].split(':')[3])
            posx = poses[0]
            posy = poses[1]
            
            agent_dict[agent_info] = (proposed_strat, (posx, posy), strat_parsed)
            
            info_garnered.append(agent_dict)
            mediate_differences(info_garnered)
            
            #if len(info_garnered) > 5 or (robot.getTime() - prev_time) > 10: 
            #   mediate_differences(info_garnered)
            #  info_garnered = []
            # prev_time = robot.getTime()
            #else: 
            #   info_garnered.append(message)
                
            receiver.nextPacket()
        
        elif 'fitness' in message: 
            fit = message.split('-')[1][7:] 
            index = int(message.split('-')[0][1:])
            partner = message.split('-')[2][5:]
            overall_fitness = message.split('-')[3][7:]
            fitness_scores[int(index)] = fit
            overall_fitness_scores[int(index)] = float(overall_fitness)

            if partner != 'none':
                new_geno = reproduce(pop_genotypes[index], pop_genotypes[partner])
                pop_genotypes[index] = new_geno
            eval_fitness(time_step)
            receiver.nextPacket()
            
        elif 'comm' in message :
            # print('processing communication request', message) 
            id_1 = message.split('-')[1]
            id_2 = message.split('-')[2]
            init_orient = float(message.split('[')[-1])
            new_2 = calc_normal(round(float(init_orient),2))
            
            comm_msg_update = 'comm_response-' + str(id_2) + "-[" + str(new_2) 

            emitter.send(str(comm_msg_update).encode('utf-8'))

            receiver.nextPacket()
            
        else: 
            receiver.nextPacket() 
            
    elif (robot.getTime() - start > simulation_time and prev_msg != 'clean finish'):
    # if over time would want to reset 
        msg = 'cleaning'
        if prev_msg != msg: 
            emitter.send('cleaning'.encode('utf-8'))
            prev_msg = msg
        while receiver.getQueueLength()>0: 
            receiver.nextPacket()
        msg = 'clean finish'
        if prev_msg != msg: 
            emitter.send('clean finish'.encode('utf-8'))
            prev_msg = msg 
        
def create_reverse_lookup_dict(dictionary):
    return {value: key for key, value in dictionary.items()}

def get_key_by_value(dictionary, value):
    reverse_lookup_dict = create_reverse_lookup_dict(dictionary)
    return reverse_lookup_dict.get(value)

def mediate_differences(msgs):
    # assuming msgs is a list of suggestions
    # ex format: [{agent_id: (strat, curr_pose)} ..] 
    # print(f'messages: {msgs}')
    global prev_msg
    global msg 
    
    cluster_list = []
    ids = []
    master_dict = {}
    agent_pos = {}
    
    for a in msgs: 
        # print(f'a in messages {a}')
        
        # for each dict of strategies in list (from each individual agent) 
        # [{'0': (1, (0.04, 0.0), '{0: (0.04, -0.04), 1: (-0.25, 0.02), 2: (-0.05, -0.07), 3: (0.19, -0.01), 4: (-0.08, -0.18)}')}]
        for key in a: 
            agent_info = a[key]
            
            info_from_strat = agent_info[2]
            curr_pose = agent_info[1]
                
            for a in info_from_strat: 
                (x,y) = info_from_strat[a]
                master_dict[(x,y)] = a 
                if (x,y) not in cluster_list: 
                    x,y = info_from_strat[a]
                    cluster_list.append((x,y))
                    
                    
    # print(f'master agent_pos: {master_dict}')
            

    cluster_merged = k_means.output_k_means_with_cluster(cluster_list, len(msgs))
    
        
    # print(f'(merged) cluster list: {cluster_merged}')
    clustered_data = {}
    proposals = []

    for i, cluster in enumerate(cluster_merged):
        strats = []
        proposals = []
        agent_pos = {}
        
        # print(f'cluster: {cluster}')
        c = cluster
        
        for pos in c: 
            agent_pos[master_dict[pos]] = pos
      
            # Find the original dictionary containing curr_pose
            for agent_dict in msgs:
                # print(f'agent_dict: {agent_dict}')
                for a, (strat, curr_pose, prop) in agent_dict.items():
                    p = prop
                    # print(f'iterating over diff items: index {a}, {strat}, {p}, pose {pos} and {curr_pose}')
                    if curr_pose == pos:
                        # print('here')
                        proposals.append(p)
                        strats.append(strat)

                        break  # Stop iterating if we found the strategy for this position
        # print(f'strats at the moment: {strats}')
        # Find the most frequently chosen strategy in the cluster
        most_common_strat = max(set(strats), key=strats.count)
        # print(f'updated agent pos: {agent_pos}')

        localMap = shared_map.LocalMap(obstacle_pos=[], obstacle_size=0.2, agent_pos=agent_pos, agent_size= 0.5, local_dim=1.0, x_bounds=(-1,1), y_bounds=(-1,1), central_loc=()) # just used to generate strat
        strat_plan = localMap.process_output(most_common_strat)
        center = localMap.calculate_center(localMap.agent_pos)

        if most_common_strat == 0: 
            # id leader 
            assigned_leader = next(key for key, value in strat_plan.items() if value == 'leader')
            # predictable movement: given curr_pos, will move directly away from center

        clustered_data[i] = [{'most_common_strat': most_common_strat, 'strat_to_use': strat_plan, 'neighbors': agent_pos, 'center': localMap.calculate_center(agent_pos)}]

        # print(f'generated strategy for cluster: {strat_plan}')
    # Example output: {0: [{'most_common_strat': 'most_common_strat1'}], ...}
    final_deliberation = clustered_data
    print(f'final deliberation: {final_deliberation}')
    
    # if prev_msg != msg:
    msg = f'final-delib:{final_deliberation}'
    message_sender(msg, individual=False)
    
    # prev_msg = msg
     
# runs simulation for designated amount of time 
def run_seconds(t,waiting=False):
    global pop_genotypes
    global fitness_scores
    global overall_fitness_scores
    global updated
    global fit_update 
    global block_list
    global start 
    global prev_msg 
    
    n = TIME_STEP / 1000*32 # convert ms to s 
    start = robot.getTime()
    new_t = round(t, 1)
    
    while robot.step(TIME_STEP) != -1:
        # run robot simulation for 30 seconds (if t = 30)
        increments = TIME_STEP / 1000
        
        if robot.getTime() - start > new_t: 
            msg = 'return_fitness'
            # if prev_msg != msg: 
            message_listener(robot.getTime()) # will clear out msg until next gen 
            print('requesting fitness')
            emitter.send('return_fitness'.encode('utf-8'))
            prev_msg = msg 
            # print('requesting fitness')
            break 

        elif not waiting: 
            # constantly checking for messages from robots 
            message_listener(robot.getTime())
                         
            if total_found == len(block_list):
                msg = 'return_fitness'
                # if prev_msg != msg: 
                emitter.send('return_fitness'.encode('utf-8'))
                prev_msg = msg 
                # print('requesting fitness')
                # print('collected all objects')
                break      
    return 
   
# will use selected partners from each robot and reproduce with that corresponding index, and update population at the end of gen          
def update_geno_list(genotype_list): 
    global fitness_scores
    global overall_fitness_scores
    global pop_genotypes 
    global gene_list
    global taken 
    global updated 
    global population 
    global pairs 
    
    # only makes executive changes if it's better off to just re-randomize population   
    # print('getting overall fitness scores --', overall_fitness_scores)
    
    # if max(overall_fitness_scores) <= 0:
    cp_genotypes = pop_genotypes.copy()
    for i in range(len(population)):
        if i not in pairs: 
            g = create_individal_genotype(gene_list)
            new_offspring = reproduce(cp_genotypes[i], cp_genotypes[i])
            # print('updated genolist --', g)
            pop_genotypes[i] = new_offspring
                 
    # update parameters to hopefully improve performance
    # print('curr population --', population, len(population))
    
    # fitness_scores = []
    fs_msg = 'fitness-scores ' + " ".join(fitness_scores)
    emitter_individual.send(fs_msg.encode('utf-8'))
    fitness_scores = ["!" for i in range(len(population))]
    overall_fitness_scores = ["!" for i in range(len(population))]
    pairs = ["!" for i in range(len(population))]
    fit_update = False 
    # print('gene pool updated') 
    updated = True

# fitness function for each individual robot 
def eval_fitness(time_step):
    global pop_genotypes 
    global fitness_scores 
    global fit_update
    global population 
    global overall_fitness_scores
            
    if '!' not in fitness_scores and '!' not in overall_fitness_scores: 
        # receiver.nextPacket()
        # print('will update gene pool --')
        fit_update = True 
        update_geno_list(pop_genotypes)

# TODO: send genotype to each individual 
def reset_genotype():
    index = 0 
    global population 
    global pop_genotypes 
    global prev_msg 
    pop_genotypes = []
    
    for i in range(len(population)):
        genotype = initial_genotypes[i]
        pop_genotypes.append(genotype)
        msg = str("#"+ str(index) + str(genotype))
        if prev_msg != msg: 
            emitter.send(str("#"+ str(index) + str(genotype)).encode('utf-8'))
            prev_msg = msg
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
    global prev_msg 
    global curr_trial 
    global phase_one_times
    global id_ind
    
    for size in robot_population_sizes:
        curr_size = size  
        initialize_genotypes(size)
        r_pos_to_generate = []

        if central: 
            generate_robot_central(size)
        else: 
            generate_robot_edge(size) # set True to switch
        
        curr_trial = 0
        if assessing and curr_trial % 2 == 0:
            regenerate_blocks(seed = 11)
        elif assessing and curr_trial % 2 != 0: 
            regenerate_blocks(seed = 15)
        else: 
            regenerate_blocks(seed = 11) 
        ind_sup = []

        for i in range(len(population)):
            ### generate supervisor for parallelization ####
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/robots/ga-individual.wbo')
            individual = rootChildrenField.getMFNode(-1)
            ind_sup.append(individual)
          
            individual.getField('translation').setSFVec3f([0, 2, 0])
            
        emitter_individual.send(id_msg.encode('utf-8'))
        emitter_individual.send(id_ind.encode('utf-8'))
        
        total_elapsed = 600
            
        num_generations = total_elapsed // simulation_time
        
        for i in range(trials): 
            print('beginning new trial', i)
            msg = 'generation-complete '+ ' '.join(pop_genotypes)
            
            # if (assessing and trial % 2) == 0 or not assessing: 
            emitter_individual.send(str(msg).encode('utf-8'))
            emitter.send(str(msg).encode('utf-8'))
            
            
            for rec_node in population: 
                r_field = rec_node.getField('rotation')
                if r_field.getSFRotation() != [0, 0, -1]:
                    r_field.setSFRotation([0, 0, -1])
                
                
            for gen in range(num_generations): 
                updated = False 
                # index = 0 
                
                print('number in population', len(population))
                print('number of genotypes',  len(pop_genotypes), 'for size: ', size)

                run_seconds(simulation_time) 
                
                for rec_node in population: 
                    r_field = rec_node.getField('rotation')
                    if r_field.getSFRotation() != [0, 0, -1]:
                        r_field.setSFRotation([0, 0, -1])
                          
                print('found genotypes')
                print('new generation starting -')
                reproduce_list = []

            overall_f.write(str(i) + ',' + str(robot.getTime()) + ',' + str(total_found) + ',' + str(size)+ ',' + 'ga' + ',' + str(20) + ',' + str(total_elapsed) + '\n')    
            overall_f.close()
            overall_f = open(f'../../graph-generation/collection-data/overall-df-{sim_type}-{curr_size}-comm_{communication}-dense_{high_dense}.csv', 'a')
            print('items collected', total_found)
            curr_trial = i + 1
            if assessing and curr_trial % 2 == 0:
                regenerate_blocks(seed = 11)
                reset_genotype() 
            elif assessing and curr_trial % 2 != 0: 
                regenerate_blocks(seed = 15)   
            else: 
                regenerate_blocks(seed = 11)
                reset_genotype() 
            
            total_found = 0 
            reproduce_list = []
            found_list = []
            msg = 'trial' + str(i)
            emitter.send(msg.encode('utf-8')) 
            prev_msg = msg
            
        for node in ind_sup: 
            node.remove() 
            
        run_seconds(5)     
         
    return 
  
def main(): 
    run_optimization()
    save_progress()
         
main()
                    







            
# ----------------------- OLD CODE ------------------------- # 
# def regenerate_blocks_dual_source():
#     global block_list
#     global r_pos_to_generate
#     global b_pos_to_generate
    
#     for obj in block_list: 
#         obj.remove()
    
#     block_list = []
    
#     if len(b_pos_to_generate) == 0: 
#         for i in range(len(r_pos_to_generate)):
#             population[i].getField('translation').setSFVec3f(r_pos_to_generate[i])
      
    
#         for i in range(20): 
#             rootNode = robot.getRoot()
#             rootChildrenField = rootNode.getField('children')
#             rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
#             rec_node = rootChildrenField.getMFNode(-1)
        
#             t_field = rec_node.getField('translation')
#             t_field.setSFVec3f([round(random.uniform(-0.5, -0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02]) 
#             block_list.append(rec_node)        
          
#         for i in range(20): 
#             rootNode = robot.getRoot()
#             rootChildrenField = rootNode.getField('children')
#             rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
#             rec_node = rootChildrenField.getMFNode(-1)
            
#             t_field = rec_node.getField('translation')
#             t_field.setSFVec3f([round(random.uniform(0.5, 0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02]) 
#             block_list.append(rec_node)    
 
#     else: 
#         # if already generated, use the previously saved positions 
#         for i in b_pos_to_generate: 
#             rootNode = robot.getRoot()
#             rootChildrenField = rootNode.getField('children')
#             rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
#             rec_node = rootChildrenField.getMFNode(-1)
        
#             t_field = rec_node.getField('translation')
#             t_field.setSFVec3f(i) 
#             block_list.append(rec_node)
               
# # creates random clustering         
# def regenerate_blocks_power_law():
#     global block_list
#     global r_pos_to_generate
#     global b_pos_to_generate
    
#     for obj in block_list: 
#         obj.remove()
    
#     block_list = []
    
#     for i in range(len(r_pos_to_generate)):
#         population[i].getField('translation').setSFVec3f(r_pos_to_generate[i])
        
        
#     if len(b_pos_to_generate) == 0: 
#         seed_file = open('../../graph-generation/seed-11-pl.csv', 'r') 
#         list = seed_file.readlines()
#         for pos in list: 
#             res = [float(i) for i in pos.strip('][\n').split(', ')]
#             b_pos_to_generate.append(res)
#             rootNode = robot.getRoot()
#             rootChildrenField = rootNode.getField('children')
#             rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
#             rec_node = rootChildrenField.getMFNode(-1)
        
#             t_field = rec_node.getField('translation')
#             t_field.setSFVec3f(res) 
#             block_list.append(rec_node)   
            
#     else: 
#         # if already generated, use the previously saved positions 
#         for i in b_pos_to_generate: 
#             rootNode = robot.getRoot()
#             rootChildrenField = rootNode.getField('children')
#             rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
#             rec_node = rootChildrenField.getMFNode(-1)
        
#             t_field = rec_node.getField('translation')
#             t_field.setSFVec3f(i) 
            
#             r_field = rec_node.getField('rotation')
#             if r_field.getSFRotation() != [0, 0, -1]:
#                 r_field.setSFRotation([0, 0, -1])
                
#             block_list.append(rec_node)
            
#     for rec_node in block_list: # set to be upright
#         r_field = rec_node.getField('rotation')
#         if r_field.getSFRotation() != [0, 0, -1]:
#             r_field.setSFRotation([0, 0, -1])
                   
# def regenerate_environment(block_dist):
#     # creates a equally distributed set of blocks 
#     # avoiding areas where a robot is already present 
#     global block_list
#     global population 
#     global r_pos_to_generate
#     global b_pos_to_generate
    
#     for obj in block_list: 
#         obj.remove()
    
#     block_list = []
    
#     for i in range(len(r_pos_to_generate)):
#         population[i].getField('translation').setSFVec3f(r_pos_to_generate[i])
        
          
#     # generates block on opposite sides of arena (randomly generated) 
#     if len(b_pos_to_generate) == 0: 
#         seed_file = open('../../graph-generation/seed-15.csv', 'r') 
#         list = seed_file.readlines()
#         for pos in list: 
#             res = [float(i) for i in pos.strip('][\n').split(', ')]
#             b_pos_to_generate.append(res)
#             rootNode = robot.getRoot()
#             rootChildrenField = rootNode.getField('children')
#             rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
#             rec_node = rootChildrenField.getMFNode(-1)
        
#             t_field = rec_node.getField('translation')
#             t_field.setSFVec3f(res) 
#             block_list.append(rec_node)  
            

#     else: 
#         # if already generated, use the previously saved positions 
#         for i in b_pos_to_generate: 
#             rootNode = robot.getRoot()
#             rootChildrenField = rootNode.getField('children')
#             rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
#             rec_node = rootChildrenField.getMFNode(-1)
        
#             t_field = rec_node.getField('translation')
#             t_field.setSFVec3f(i) 
#             block_list.append(rec_node)
            
#     # seed_11.close()
    
#     for rec_node in block_list: # set to be upright
#         r_field = rec_node.getField('rotation')
#         if r_field.getSFRotation() != [0, 0, -1]:
#             r_field.setSFRotation([0, 0, -1])   

# initially reads from file for re-producibility, and then continues to re-read (will be second)        
# def regenerate_environment_alternate(block_dist): # will stay constant based off seed 
#     # creates a equally distributed set of blocks 
#     # avoiding areas where a robot is already present 
#     global block_list
#     global population 
#     global r_pos_to_generate
#     global b_pos_to_generate_alternative
    
#     for obj in block_list: 
#         obj.remove()
    
#     block_list = []
    
#     for i in range(len(r_pos_to_generate)):
#         population[i].getField('translation').setSFVec3f(r_pos_to_generate[i])
        
#     # generates block on opposite sides of arena (randomly generated) 
#     if len(b_pos_to_generate_alternative) == 0: 
#         seed_file = open('../../graph-generation/seed-15-pl.csv', 'r') 
#         list = seed_file.readlines()
#         for pos in list: 
#             res = [float(i) for i in pos.strip('][\n').split(', ')]
#             b_pos_to_generate_alternative.append(res)
#             rootNode = robot.getRoot()
#             rootChildrenField = rootNode.getField('children')
#             rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
#             rec_node = rootChildrenField.getMFNode(-1)
        
#             t_field = rec_node.getField('translation')
#             t_field.setSFVec3f(res) 
#             block_list.append(rec_node)   
        
#     else: 
#         # if already generated, use the previously saved positions 
#         for i in b_pos_to_generate_alternative: 
#             rootNode = robot.getRoot()
#             rootChildrenField = rootNode.getField('children')
#             rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
#             rec_node = rootChildrenField.getMFNode(-1)
        
#             t_field = rec_node.getField('translation')
#             t_field.setSFVec3f(i) 
#             block_list.append(rec_node)
               
# def regenerate_blocks_random(): 

#     global block_list
#     for obj in block_list: 
#         obj.remove()
    
#     block_list = []
    
#     # generates block on opposite sides of arena (randomly generated) 
#     for i in range(10): 
#         rootNode = robot.getRoot()
#         rootChildrenField = rootNode.getField('children')
#         rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
#         rec_node = rootChildrenField.getMFNode(-1)
    
#         t_field = rec_node.getField('translation')
#         t_field.setSFVec3f([round(random.uniform(0.9, -0.9),2), round(random.uniform(0.3, 0.85),2), 0.02]) 
#         block_list.append(rec_node)
    
#     for i in range(10): 
#         rootNode = robot.getRoot()
#         rootChildrenField = rootNode.getField('children')
#         rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
#         rec_node = rootChildrenField.getMFNode(-1)
    
#         t_field = rec_node.getField('translation')
#         t_field.setSFVec3f([round(random.uniform(0.9, -0.9),2), round(random.uniform(-1, 0.23),2), 0.02]) 
#         block_list.append(rec_node)
        
    
# def regenerate_blocks_single_source():
#     global block_list
#     global r_pos_to_generate
#     global b_pos_to_generate
    
#     for obj in block_list: 
#         obj.remove()
    
#     block_list = []
    
#     for i in range(len(r_pos_to_generate)):
#         population[i].getField('translation').setSFVec3f(r_pos_to_generate[i])
        
#     if len(b_pos_to_generate) == 0: 
#         for i in range(40): 
#             rootNode = robot.getRoot()
#             rootChildrenField = rootNode.getField('children')
#             rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
#             rec_node = rootChildrenField.getMFNode(-1)
        
#             t_field = rec_node.getField('translation')
#             t_field.setSFVec3f([round(random.uniform(-0.5, -0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02]) 
#             block_list.append(rec_node)
            
#     else: 
#         # if already generated, use the previously saved positions 
#         for i in b_pos_to_generate: 
#             rootNode = robot.getRoot()
#             rootChildrenField = rootNode.getField('children')
#             rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
#             rec_node = rootChildrenField.getMFNode(-1)
        
#             t_field = rec_node.getField('translation')
#             t_field.setSFVec3f(i) 
#             block_list.append(rec_node)