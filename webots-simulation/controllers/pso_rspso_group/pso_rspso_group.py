from controller import Supervisor, Node, Keyboard, Emitter, Receiver, Field
import random
import math 

"""
Main supervisor base 
Optimization algorithm - Collaboration-oriented 
Angel Sylvester 2022
"""
# # collected counts csv generation 
overall_f = open('../../graph-generation/collection-data/overall-pso-rdpso-info.csv', 'a') 

# set-up robot2
TIME_STEP = 32
robot = Supervisor()  # create Supervisor instance
# emitter to send info to robots 
emitter = robot.getDevice("emitter")
emitter.setChannel(4)
# set receiver to receive info 
receiver = robot.getDevice("receiver") 
receiver.enable(TIME_STEP)
receiver.setChannel(5) 

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
sc_max = 30 # max number of time before punishment (might not be fine-tuned)
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
n_kills = 0 
trial_count = 0

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
overall_population = []
prev_msg = ""
random.seed(11)
assessing = False 
repopulate = False
success = False 
size = 5 
 
group_id = int(robot.getName()[11:-1]) - 1
#### allow personal supervisor to send important encounter info back to supervisor ## 
emitter = robot.getDevice("emitter")
emitter.setChannel(4)

#### receive info from supervisor regarding robot #####
receiver = robot.getDevice("receiver") 
receiver.enable(TIME_STEP)
receiver.setChannel(5) 

### allow personal supervisor to recieve encounter info from robot 
receiver_individual = robot.getDevice("receiver_processor") 
receiver_individual.enable(TIME_STEP)
receiver_individual.setChannel(int(group_id * 10)) # will be updated to be robot_id * 10 
emitter_individual = robot.getDevice("emitter_processor")
emitter_individual.setChannel((int(group_id) * 10) + 1)

# must possess connection to overal supervisor (to be actually generated) & connection to each robot 
# then adds collection statistics that are unique to given trial 
# trial info will be sent with rec node information 
# all robot rec nodes will be provided 

# if group id is -1, must be able to send rec node of best performing agent and remove from 
# current list 

# must possess connection to socially excluded group (which will be generated by overall supervisor) 

robots_in_group = []
id_name_dic = {}

# for PSO, after end of each iteration, recalculate global and local best for each agent #
def calc_global_local():
    # will send bulk msg to be parsed by robot (index correspond to robot id) 
    # will use collected count to determine best and local best 
    global population 
    global collected_count
    global group_id
    global group_dic
    
    msg = ""
    index = 0
    
    print('group dic', group_dic, group_id)
    
    if len(group_dic) != 0: 
        global_best = robot.getFromId(int(max(group_dic, key=group_dic.get))).getField('translation')
         
        for r in population: 
            t_field = robot.getFromId(int(r)).getField('translation').getSFVec3f()
            neighs = find_neighbor(r, t_field) 
            # loc_best = max(collected_count[neighs[0]], collected_count[neighs[1]])
            # pos = population[loc_best].getField('translation')
            given_id = overall_population.index(int(r))
                
            msg += str(given_id) + '%' + str(neighs[0]) + '%' + str(neighs[0]) + '*'
            index += 1 
            
        msg += str(global_best.getSFVec3f())
        print('pso calcs', group_id, msg) 
        emitter_individual.send(str("pso-" + str(msg)).encode('utf-8'))    

 
def find_neighbor(curr_r, curr_loc): 
    global population 
    neighs = [] # will correspond to robot id 
    dists = []
    
    
    for r in population: 
    
        if curr_r != r: 
            t_field = robot.getFromId(r).getField('translation').getSFVec3f()
            print(curr_loc, t_field) 
            d = math.dist(curr_loc, t_field)
            dists.append(d) 
            
        else: 
            dists.append(math.inf) # super large to make sure 
            
        
    # find min and min2 of list and corresponding index 
    while len(neighs) != 1: # finding top 2 
        
        min_n = min(dists) 
        index = dists.index(min_n)
        dists[index] = math.inf # remove as candidate 
        
        neighs.append(index)
        
    return [robot.getFromId(population[index]).getField('translation').getSFVec3f()[0], robot.getFromId(population[index]).getField('translation').getSFVec3f()[1]]

def search_counter():
    global sc_max
    global simulation_time
    global num_excluded
    global group_dic 
    
    if len(group_dic) != 0: 
        simulation_time = sc_max*(1 - (1 / (num_excluded + 1)))
        print('updated simulation time', simulation_time) 

def get_best_in_excluded(select):
    global group_dic
    global population 
    global n_i 
    
    print('getting best in excluded', group_dic, group_id)
    
    new_group = ''
    size = 0 
    if select == 1: 
        thres = 1
    else: 
        thres = n_i 
        
    while size < thres or len(population) > 0: 
        r = robot.getFromId(int(max(group_dic, key=group_dic.get)))
        
        given_id = overall_population.index(max(group_dic, key=group_dic.get))
            
        new_group += str(r.getId()) + '*' 
        del group_dic[r.getId()]
        population.remove(r)
        size += 1
        
    return new_group 
    
def get_worst_in_group():
    global group_dic
    global population 
    global n_i 
    global n_min 
    
    new_group = ''
    size = 0 
    while size < 1 and len(population) > 0:
        print(group_dic, population)
        
        r = robot.getFromId(int(min(group_dic, key=group_dic.get)))
        given_id = overall_population.index(min(group_dic, key=group_dic.get))
            
        new_group += str(int(min(group_dic, key=group_dic.get))) + '%' 
        population.remove(int(min(group_dic, key=group_dic.get)))
        del group_dic[int(min(group_dic, key=group_dic.get))]
        print('worst', group_dic, group_id)
        size += 1
        
    if len(population) < n_min:
        for r in population: 
            given_id = str(int(min(group_dic, key=group_dic.get)))
            
            new_group += str(given_id) + '%' 
            
        print('removing to new', group_dic, group_id)
        
    return new_group 
    
    
def reward_subswarm():
    # incorporate new swarm generation if num time rewarded > penalized 
    global spawn_prob
    global reward_count 
    global punish_count
    global size
    
    if group_id != 0: 
        # send request to id 
        # will get best from excluded group 
        request = 'rewarded*' + str(group_id)
        emitter.send(request.encode('utf-8'))
        
        if (reward_count - punish_count) > 0: 
            spawn_prob = random.uniform(0,1) / size 
            choice = random.choices([0,1], [spawn_prob, 1 - spawn_prob])
            if choice == 0: # will generate sub-swarm 
                rootNode = robot.getRoot()
                rootChildrenField = rootNode.getField('children')
                rootChildrenField.importMFNode(-1, '../las_supervisor/robots/ga-pso-rpso-group.wbo')
                individual = rootChildrenField.getMFNode(-1)
                individual.getField('translation').setSFVec3f([0, 2, 0])
                
                
                # want to send to overall supervisor to send too all other agents 
                msg = 'sub-swarm' + str(int(individual.getName()[11:-1]) - 1)
                emitter.send(msg.encode('utf-8'))
                
                 # create another supervisor group (+1 from previous) 
                 # send info to -1 so that this information can be sent to new supervisor and individual controllers are updated 

    
    
def punish_subswarm():
    global num_excluded
    global reward_count 
    global punish_count
    global group_id
    
    if group_id != 0: 
        # find worst performing and send to -1 
        # r = robot.getId()
        # request = 'punished*' + str(group_id) + '*' + str(r)
        # emitter_individual.send(request.encode('utf-8'))
        
        num_excluded += 1 
        punish_count += 1 
        search_counter() # sets simulation time accordingly
        
        # if (punish_count - reward_count) > 7:
        w = get_worst_in_group()
        print('punished', group_id, w)
        msg = 'punished*' + str(w) + '*' + str(group_id)
        emitter_individual.send(msg.encode('utf-8'))
        emitter.send(msg.encode('utf-8'))
            # if size of sub swarm less than n_min, send agents to -1 and request overall supervisor to delete it completely 
    
    pass 
            

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
    global size
    global reward_count 
    global punish_count
    global trial_count 
    global success 
    global overall_population
    global group_id
    global population
    global group_dic

    if receiver.getQueueLength()>0 and (robot.getTime() - start < simulation_time):
        message = receiver.getData().decode('utf-8')
        print('group messages ', message)
        if message[0] == "$": # handles deletion of objects when grabbed
            obj_node = robot.getFromId(int(message.split("-")[1]))
            given_id = message.split("-")[0][1:]
            group_id = message.split("-")[2]
            # print(obj_node)
            if obj_node is not None:
                
                # r_node_loc = population[int(message.split("-")[0][1:])].getField('translation').getSFVec3f()
                # t_field = obj_node.getField('translation')
                # t_node_loc = t_field.getSFVec3f()
                
                # print(math.dist(r_node_loc, t_node_loc))
                if (math.dist(r_node_loc, t_node_loc) < 0.15): # only count if actually in range 
                    # if repopulate: 
                        # will be placed somewhere random 
                        # side = random.randint(0,1)
                        # if side == 1:
                            # t_field.setSFVec3f([round(random.uniform(-0.5, -0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02]) 
                        # else: 
                            # t_field.setSFVec3f([round(random.uniform(0.5, 0.9),2), round(random.uniform(-0.9, 0.9),2), 0.02])   
                    # else:
                        # t_field.setSFVec3f([-0.9199,-0.92, 0.059]) 
  
                    # obj_node.remove()
                    # remove redundant requests 
                    if obj_node not in found_list:
                        total_found += 1
                        reward_count += 1
                        found_list.append(obj_node)
                        reward_subswarm()
                        r = robot.getFromId(int(given_id))
                        group_dic[given_id] += 1
                        success = True
                
                        collected_count[int(message.split("-")[0][1:])] = collected_count[int(message.split("-")[0][1:])] + 1
                        msg_info = "%" + message[1:]
                        if prev_msg != msg_info: 
                            emitter.send(str(msg_info).encode('utf-8'))
                            emitter_individual('group update' + str(max(group_dic).getField('translation').getSFVec3f()).encode('utf-8')) # allow ind to keep track of best 
                            prev_msg = msg_info
                            print('removing object') 

            receiver.nextPacket()
          
        elif 'sub-swarm' in message and group_id == 0:
            best_n = get_best_in_excluded() # will get given ids (broken up by *) 
            # send to agents that are relevant 
            msg = 'subbed*' + str(best_n) + '*' + str(message[9:]) 
            
            emitter_individual.send(msg.encode('utf-8')) # will send to individual robot 
            emitter.send(str(msg).encode('utf-8')) # will send to supervisor to update team layout 
            
            # will find best (atmost initialized value, and create new supervisor) 
            # get group id and robot id num to send as msg, will be changed to new supervisor group id 
            receiver.nextPacket()
            
        elif 'robot-info' in message: 
            size = message.split('*')[1]
            trial_count = message.split('*')[2]
            receiver.nextPacket()
            
        elif 'punished' in message and group_id == 0: 
            # remove worst and send to 0 
            given_robot = message.split('*')[1].split('%')
            gp_id = message.split('*')[-1]
            print('adding removed items to 0')
            # msg = 'punished*' + str(w) + '*' + str(group_id)
            
            if group_id == gp_id:
                for g in given_robot: 
                    group_dic[g] = 0 
                    population.append(int(g))
            
            
            receiver.nextPacket() 
            
        elif 'rewarded' in message and group_id == 0: 
            destination = message.split('*')[1]
            best_n = get_best_in_excluded()
            print('found excluded to add', best_n) 
            msg = 'subbed*' + str(best_n) + '*' + str(destination) 
            
            emitter_individual.send(msg.encode('utf-8')) # will send to individual robot 
        
            receiver.nextPacket()
            
        elif 'subbed' in message: 
            dest = message.split('*')[-1]
            r = message.split('*')[1]
            if int(dest) == group_id: 
                group_dic[int(r)] = 0
                population.append(int(r)) # get robot form id 
          
            receiver.nextPacket()   
        
        elif 'ids' in message: 
            id_msg = message.split(" ")[1:]
            
            for id in id_msg: # will convert to nodes to eventual calculation 
                node = robot.getFromId(int(id))
                overall_population.append(int(id))
 
            receiver.nextPacket()   
            
        elif 'group' in message: # assigns group 
            assignments = message[5:] 
            ind = 0

            for a in assignments: 
                if int(a) == int(group_id):
                    population.append(overall_population[ind]) 
                    group_dic[overall_population[ind]] = 0
                ind += 1 
            print('initial group;', group_dic)
            receiver.nextPacket()
          
        elif 'fitness' in message:
            print('message', message, group_dic) 
            fit = message.split('-')[1][7:] 
            index = message.split('-')[0][1:]
            fitness_scores[int(index)] = fit
            print('fitness scores', fitness_scores)
            
            eval_fitness(time_step)
            
            receiver.nextPacket()
            # will be generalized 

        else: 
            receiver.nextPacket()
        
    # send from individual robots belonging to respective group 
    # if receiver_individual.getQueueLength()>0:  
        
    
    
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
    global success
    global size 

    
    # for i in range(trials): 
        # print('beginning new trial', i)
    simulation_time = sc_max
    spawn_prob = random.uniform(0,1) / size 
    success = False 
    total_time_elapsed = 0
    
    while total_time > total_time_elapsed: 
        
        updated = False     
        run_seconds(simulation_time) 
        
        # if unsuccessful, punish 
        if not success: 
            punish_subswarm()
            print('given group', group_id, 'failed')
            
        print('overall groups', group_id, group_dic)
        calc_global_local() # updates each robot to pivot toward successful place 
        search_counter()
        total_time_elapsed += simulation_time
        
        # print('waiting for genotypes')
        
        run_seconds(5, True) # time to re-orient 
        
        for rec_node in population: 
            r_field = rec_node.getField('rotation')
            if r_field.getSFRotation() != [0, 0, -1]:
                r_field.setSFRotation([0, 0, -1])
        
        # regenerate_blocks_power_law()
        # regenerate_blocks_single_source()
        # regenerate_blocks_dual_source()
    total_found = 0 
    found_list = [] 
    index = 0 
    num_excluded = 0 
    
    # will be deleted for subsequent trial 
    emitter.send('trial_complete'.encode('utf-8')) 
   
    return 

def main(): 
    # restore_positions()
    # initialize_genotypes()
    run_optimization()
    # save_progress()
main()
                    
            
            
