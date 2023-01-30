from controller import Supervisor, Node, Keyboard, Emitter, Receiver, Field
import random
import math 

"""
Main supervisor base 
Optimization algorithm - Collaboration-oriented 
Angel Sylvester 2022
"""
# # collected counts csv generation 
overall_f = open('../../graph-generation/collection-data/overall-pso-rdpso-info.csv', 'w') 
overall_f.write('trial' + ',time' + ',objects retrieved' + ',size'+ ',type' + '\n')
overall_f.close()
overall_f = open('../../graph-generation/collection-data/overall-pso-rdpso-info.csv', 'a') 

# individual robot collision counts 
strategy_f = open("../../graph-generation/collision-data/pso-rdpso-info.csv", 'w')
strategy_f.write('agent id,'+ 'time step,' +' time since last block' + ',size' + ',collisions'+ ',type' + ',collected' + '\n')
strategy_f.close()

# set-up robot2
TIME_STEP = 32
robot = Supervisor()  # create Supervisor instance
# emitter to send info to robots 
emitter = robot.getDevice("emitter")
emitter.setChannel(1)
# set receiver to receive info 
receiver = robot.getDevice("receiver") 
receiver.enable(TIME_STEP)
receiver.setChannel(2) 

# set up timing so consistent with ga 
start = 0
num_generations = 60
total_time = 600 
trials = 50
simulation_time = 10
robot_population_sizes = [5, 10, 15] # [5, 10, 15]
curr_size = robot_population_sizes[0]
curr_trial = 0 
population = []
initial_genotypes = []
fitness_scores = []
pop_genotypes = []
sc_max = 50 # max number of time before punishment (might not be fine-tuned)
num_excluded = 0
spawn_prob = 0 
reward_count = 0 
punish_count = 0 
num_swarms = 5 
c_1 = 0.89
c_2 = 0.8
c_3 = 0.9 
n_i = 2
n_min = 1 
nmax = 6
group_dic = {}

# sim statistics 
total_collected = 0 
taken = False # getting second child assigned 
total_found = 0
updated = False
fit_update = False 
found_list = []
block_list = []
arena_area = robot.getFromDef("arena")
collected_count = []
r_pos_to_generate = []
b_pos_to_generate = []
b_pos_to_generate_alternative = []
prev_msg = ""
random.seed(11)
assessing = False 
repopulate = True

def search_counter():
    global sc_max
    global simulation_time
    global num_excluded
    
    simulation_time = sc_max*(1 - (1 / (num_excluded + 1)))


# for PSO, after end of each iteration, recalculate global and local best for each agent #
def calc_global_local():
    # will send bulk msg to be parsed by robot (index correspond to robot id) 
    # will use collected count to determine best and local best 
    global population 
    global collected_count
    
    msg = ""
    index = 0
    
    global_best = population[collected_count.index(max(collected_count))].getField('translation')
     
    for r in population: 
        t_field = r.getField('translation')
        neighs = find_neighbor(r, t_field) 
        loc_best = max(collected_count[neighs[0]], collected_count[neighs[1]])
        pos = population[loc_best].getField('translation')
        msg += str(pos) + '*'
        index += 1 
        
    msg += str(global_best)
    emitter.send(str("pso-" + str(msg)).encode('utf-8'))    
 
def find_neighbor(curr_r, curr_loc): 
    global population 
    neighs = [] # will correspond to robot id 
    
    robo_id = 0 
    dists = []
    for r in population: 
    
        if curr_r != r: 
            t_field = rec_node.getField('translation')
            d = math.dist(curr_loc, t_field)
            dists.append(d) 
            
        else: 
            dists.append(math.inf) # super large to make sure 
            
        robo_id += 1 
        
    # find min and min2 of list and corresponding index 
    while len(neighs) != 1: # finding top 2 
        
        min = min(dists) 
        index = dists.index(min)
        dists[index] = math.inf # remove as candidate 
        
        neighs.append(index)
        
    return neighs 
        

def generate_robot_central(num_robots):
    global fitness_scores 
    global collected_count 
    global population 
    global r_pos_to_generate
    global n_i 
    global n_min = 1 
    global nmax = 6
    global group_dic 
    
    # initialize_genotypes(num_robots)
    emitter.send(str("size-" + str(num_robots)).encode('utf-8'))

    if len(population) != 0: 
    
        for r in population: 
            r.remove()
    
    population = []
    fitness_scores = []
    collected_count = []
    
    num_whole_groups = num_robots // n_i 
    g_index = 0 
    msg = "" 
    
    for i in range(num_robots):
    
        if g_index == num_whole_groups:
            g_index = 0 
            
        rootNode = robot.getRoot()
        rootChildrenField = rootNode.getField('children')
        rootChildrenField.importMFNode(-1, '../las_supervisor/robots/robot-pso-rdpso.wbo') 
        rec_node = rootChildrenField.getMFNode(-1)
    
        t_field = rec_node.getField('translation')
        pos = [round(random.uniform(0.25, -0.25),2), round(random.uniform(0.25, -0.25) ,2), 0.02]
        r_pos_to_generate.append(pos)
        t_field.setSFVec3f(pos)
        
        # sets up metrics 
        fitness_scores.append("!")
        collected_count.append(0)
        population.append(rec_node)
        
        # will allocate robot to each group 
        msg = str(g_index) + '*' 
        
        
    emitter.send('group' + str(msg).encode('utf-8'))
    
    
def reward_subswarm():
    # incorporate new swarm generation if num time rewarded > penalized 
    global spawn_prob
    global reward_count 
    global punish_count
    
    
    # incorporate swarm deletion (if num times penalized several)
    
    pass 
    
def punish_subswarm():
    pass 

def regenerate_environment(block_dist):
    # creates a equally distributed set of blocks 
    # avoiding areas where a robot is already present 
    global block_list
    global population 
    global r_pos_to_generate
    global b_pos_to_generate
    
    for obj in block_list: 
        obj.remove()
    
    block_list = []
    
    for i in range(len(r_pos_to_generate)):
        population[i].getField('translation').setSFVec3f(r_pos_to_generate[i])
    
        
    # generates block on opposite sides of arena (randomly generated) 
    if len(b_pos_to_generate) == 0: 
        for i in range(10): 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            pose = [round(random.uniform(0.9, -0.9),2), round(random.uniform(0.3, 0.85),2), 0.02]
            t_field.setSFVec3f(pose) 
            b_pos_to_generate.append(pose)
            block_list.append(rec_node)
        
        for i in range(10): 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            pose = [round(random.uniform(0.9, -0.9),2), round(random.uniform(-1, 0.23),2), 0.02]
            t_field.setSFVec3f(pose) 
            b_pos_to_generate.append(pose)
            block_list.append(rec_node)
    else: 
        # if already generated, use the previously saved positions 
        for i in b_pos_to_generate: 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            t_field.setSFVec3f(i) 
            block_list.append(rec_node)

        
# initially reads from file for re-producibility, and then continues to re-read (will be second)        
def regenerate_environment_alternate(block_dist): # will stay constant based off seed 
    # creates a equally distributed set of blocks 
    # avoiding areas where a robot is already present 
    global block_list
    global population 
    global r_pos_to_generate
    global b_pos_to_generate_alternative
    
    for obj in block_list: 
        obj.remove()
    
    block_list = []
    
    for i in range(len(r_pos_to_generate)):
        population[i].getField('translation').setSFVec3f(r_pos_to_generate[i])
        
    # generates block on opposite sides of arena (randomly generated) 
    if len(b_pos_to_generate_alternative) == 0: 
        seed_file = open('../../graph-generation/seed-15.csv', 'r') 
        list = seed_file.readlines()
        for pos in list: 
            res = [float(i) for i in pos.strip('][\n').split(', ')]
            b_pos_to_generate_alternative.append(res)
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            t_field.setSFVec3f(res) 
            block_list.append(rec_node)   
        
    else: 
        # if already generated, use the previously saved positions 
        for i in b_pos_to_generate_alternative: 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            t_field.setSFVec3f(i) 
            block_list.append(rec_node)
                  
def regenerate_blocks_single_source():
    global block_list
    global r_pos_to_generate
    global b_pos_to_generate
    
    for obj in block_list: 
        obj.remove()
    
    block_list = []
    
    for i in range(len(r_pos_to_generate)):
        population[i].getField('translation').setSFVec3f(r_pos_to_generate[i])
        
    if len(b_pos_to_generate) == 0: 
        for i in range(40): 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            t_field.setSFVec3f([round(random.uniform(-0.5, -0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02]) 
            block_list.append(rec_node)
            
    else: 
        # if already generated, use the previously saved positions 
        for i in b_pos_to_generate: 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            t_field.setSFVec3f(i) 
            block_list.append(rec_node)
        
def regenerate_blocks_dual_source():
    global block_list
    global r_pos_to_generate
    global b_pos_to_generate
    
    for obj in block_list: 
        obj.remove()
    
    block_list = []
    
    if len(b_pos_to_generate) == 0: 
        for i in range(len(r_pos_to_generate)):
            population[i].getField('translation').setSFVec3f(r_pos_to_generate[i])
            
    
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
 
    else: 
        # if already generated, use the previously saved positions 
        for i in b_pos_to_generate: 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            t_field.setSFVec3f(i) 
            block_list.append(rec_node)
               
# creates random clustering         
def regenerate_blocks_power_law():
    global block_list
    global r_pos_to_generate
    global b_pos_to_generate
    
    for obj in block_list: 
        obj.remove()
    
    block_list = []
    
    for i in range(len(r_pos_to_generate)):
        population[i].getField('translation').setSFVec3f(r_pos_to_generate[i])
        
    x_min = 1
    alpha = 2.5
    centers = []
    
    
    if len(b_pos_to_generate) == 0: 

        for i in range(40): # will be number of clusters instead of disparate blocks 
            r = random.random()
            x_smp = round(x_min * (1 - r) ** (-1 / (alpha - 1)))
            print(x_smp)
            if x_smp > 5: 
                x_smp = 5 
                
            center = random.choices([[round(random.uniform(0.9, -0.9),2), round(random.uniform(0.3, 0.85),2), 0.02], [round(random.uniform(0.9, -0.9),2), round(random.uniform(-1, 0.23),2), 0.02]])[0]
            other = center
            if len(centers) != 0:
                while all(math.dist(center, other) < 0.2 and math.dist(center, (0,0,0.02)) < 0.8 for other in centers): # generate center until appropriate distance away 
                     center = random.choices([[round(random.uniform(0.9, -0.9),2), round(random.uniform(0.3, 0.85),2), 0.02], [round(random.uniform(0.9, -0.9),2), round(random.uniform(-1, 0.23),2), 0.02]])[0]
            else: 
                while math.dist(center, (0,0,0.02)) < 0.8: # generate center until appropriate distance away 
                     center = random.choices([[round(random.uniform(0.9, -0.9),2), round(random.uniform(0.3, 0.85),2), 0.02], [round(random.uniform(0.9, -0.9),2), round(random.uniform(-1, 0.23),2), 0.02]])[0]
            centers.append(center)
    
            x, y, z = center[0],center[1], center[2]
            x_smp -= 1
            pot = []
            new_c = [x+0.05, y, z]
            new_c1 = [x, y + 0.05, z]
            new_c2 = [x-0.05, y, z]
            new_c3 = [x, y - 0.05, z]
            pot.append(new_c)
            pot.append(new_c1)
            pot.append(new_c2)
            pot.append(new_c3)
            
            for i in range(x_smp): # will be clumped in same location
                rootNode = robot.getRoot()
                rootChildrenField = rootNode.getField('children')
                rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
                rec_node = rootChildrenField.getMFNode(-1)
            
                t_field = rec_node.getField('translation')
                
                # want to have additional parts centralized (at most 4 others) 
    
                t_field.setSFVec3f(pot[i]) 
                b_pos_to_generate.append(pot[i])
                block_list.append(rec_node)
                
            
    else: 
        # if already generated, use the previously saved positions 
        for i in b_pos_to_generate: 
            rootNode = robot.getRoot()
            rootChildrenField = rootNode.getField('children')
            rootChildrenField.importMFNode(-1, '../las_supervisor/cylinder-obj.wbo') 
            rec_node = rootChildrenField.getMFNode(-1)
        
            t_field = rec_node.getField('translation')
            t_field.setSFVec3f(i) 
            block_list.append(rec_node)
            
    for rec_node in block_list: # set to be upright
        r_field = rec_node.getField('rotation')
        if r_field.getSFRotation() != [0, 0, -1]:
            r_field.setSFRotation([0, 0, -1])
            

def save_progress():
    global overall_f
    
    print('progress saved to csv')
    emitter.send('sim-complete'.encode('utf-8'))

def message_listener(time_step):
    global total_found
    global found_list
    global block_list
    global collected_count 
    global curr_size 
    global start 
    global prev_msg 
    global simulation_time
    global prev_msg
    global group_dic

    if receiver.getQueueLength()>0 and (robot.getTime() - start < simulation_time):
        message = receiver.getData().decode('utf-8')
        if message[0] == "$": # handles deletion of objects when grabbed
            obj_node = robot.getFromId(int(message.split("-")[1]))
            given_id = message.split("-")[0][1:]
            group_id = message.split("-")[2]
            # print(obj_node)
            if obj_node is not None:
                r_node_loc = population[int(message.split("-")[0][1:])].getField('translation').getSFVec3f()
                t_field = obj_node.getField('translation')
                t_node_loc = t_field.getSFVec3f()
                
                # print(math.dist(r_node_loc, t_node_loc))
                if (math.dist(r_node_loc, t_node_loc) < 0.15): # only count if actually in range 
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
                        found_list.append(obj_node)
                        collected_count[int(message.split("-")[0][1:])] = collected_count[int(message.split("-")[0][1:])] + 1
                        msg_info = "%" + message[1:]
                        if prev_msg != msg_info: 
                            emitter.send(str(msg_info).encode('utf-8'))
                            prev_msg = msg_info
                            print('removing object') 

            receiver.nextPacket()
            
        elif 'fitness' in message:
            print('message', message) 
            fit = message.split('-')[1][7:] 
            index = message.split('-')[0][1:]
            fitness_scores[int(index)] = fit
            print('fitness scores', fitness_scores)
            
            eval_fitness(time_step)
            
            receiver.nextPacket()
            # will be generalized 
            
        else: 
            receiver.nextPacket()
            
    elif (robot.getTime() - start > simulation_time and prev_msg != 'clean finish'):
    # if over time would want to reset 
        emitter.send('cleaning'.encode('utf-8'))
        
        while receiver.getQueueLength()>0:
            receiver.nextPacket()
            
        emitter.send('clean finish'.encode('utf-8'))
        prev_msg = 'clean finish'

# runs simulation for designated amount of time 
def run_seconds(t,waiting=False):
    global pop_genotypes
    global fitness_scores
    global updated
    global fit_update 
    global start 
    global prev_msg 
    
    n = TIME_STEP / 1000*32 # convert ms to s 
    start = robot.getTime()
    
    new_t = round(t, 1)
    
    while robot.step(TIME_STEP) != -1:
        # run robot simulation for 30 seconds (if t = 30)
        increments = TIME_STEP / 1000
        
        # if waiting:
            # if not updated:
                # message_listener(robot.getTime())
                # eval_fitness(robot.getTime())
                # continue 
            # else: 
                # break 
        
        if robot.getTime() - start > new_t: 
            message_listener(robot.getTime())
            emitter.send('return_fitness'.encode('utf-8'))
            prev_msg = 'return_fitness' 
            print('requesting fitness')
            break 

        elif not waiting: 
            # constantly checking for messages from robots 
            message_listener(robot.getTime())
                         
            if total_found == len(block_list) and len(block_list) != 0:
                emitter.send('return_fitness'.encode('utf-8'))
                prev_msg = 'return_fitness' 
                print('collected all objects')
                break      
    return 
            
def update_geno_list(): 
    global fitness_scores
    global taken 
    global updated 
                     
    # update parameters to hopefully improve performance
    fitness_scores = ["!" for i in range(len(population))]
    fit_update = False 
    print('gene pool updated', fitness_scores) 
    updated = True

# fitness function for each individual robot 
def eval_fitness(time_step):
    global fitness_scores 
    global fit_update
    global updated
    
    # print('evaluating fitness', fitness_scores)
            
    if '!' not in fitness_scores: 
        # receiver.nextPacket()
        print('will update gene pool --')
        fit_update = True 
        update_geno_list()

def run_optimization():
    global gene_list 
    global updated
    global simulation_time 
    global total_found 
    global overall_f
    global found_list
    global r_pos_to_generate
    global curr_size
    global population 
    global spawn_prob 
    global nmax

    for size in robot_population_sizes: 
        nmax = size
        # initialize_genotypes(size)
                # creates a csv specific to the robot 
        curr_size = size
        r_pos_to_generate = []
        generate_robot_central(size)
        
        curr_trial = 0 
        
        if assessing and curr_trial % 2 == 0:
            regenerate_environment(0.2)
            # regenerate_environment_alternate(0.2) 
        elif assessing and curr_trial % 2 != 0: 
            regenerate_environment_alternate(0.2)    
        else: 
            regenerate_environment(0.2)
        # regenerate_blocks_power_law()
        # regenerate_blocks_single_source()
        # regenerate_blocks_dual_source()
        
        for rec_node in population: 
            r_field = rec_node.getField('rotation')
            if r_field.getSFRotation() != [0, 0, -1]:
                r_field.setSFRotation([0, 0, -1])
        
        total_time_elapsed = 0 
        
        for i in range(trials): 
            print('beginning new trial', i)
            spawn_prob = random.uniform(0,1) / size 
            
            
            while total_time > total_time_elapsed: 
                
                updated = False     
                run_seconds(simulation_time) 
                calc_global_local() # updates each robot to pivot toward successful place 
                search_counter()
                total_time_elapsed += simulation_time
                
                # print('waiting for genotypes')
                
                run_seconds(5, True) # time to re-orient 
                
                for rec_node in population: 
                    r_field = rec_node.getField('rotation')
                    if r_field.getSFRotation() != [0, 0, -1]:
                        r_field.setSFRotation([0, 0, -1])
                
                print('found genotypes')
                print('new generation starting -')
                reproduce_list = []
                # generate_robot_central(size)
                # regenerate_environment(0.2)  
            
            overall_f.write(str(i) + ',' + str(robot.getTime()) + ','  + str(total_found) + ','  + str(size)+ ',crw' + '\n')    
            overall_f.close()
            overall_f = open('../../graph-generation/collection-data/overall-pso-rdpso-info.csv', 'a') 
            print('items collected', total_found)
            curr_trial = i + 1  
            if assessing and curr_trial % 2 == 0:
                regenerate_environment(0.2)
            elif assessing and curr_trial % 2 != 0: 
                regenerate_environment_alternate(0.2)    
            else: 
                regenerate_environment(0.2)
            # regenerate_blocks_power_law()
            # regenerate_blocks_single_source()
            # regenerate_blocks_dual_source()
            total_found = 0 
            found_list = [] 
            index = 0 
            emitter.send('trial_complete'.encode('utf-8')) 
            # TODO: delete all robots and group supervisor for next sim 
                
    overall_f.close()
    emitter.send('sim-complete'.encode('utf-8'))    
    return 

def main(): 
    # restore_positions()
    # initialize_genotypes()
    run_optimization()
    # save_progress()
main()
                    
            
            
