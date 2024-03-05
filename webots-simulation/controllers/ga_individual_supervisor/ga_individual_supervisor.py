from controller import Supervisor, Node, Keyboard, Emitter, Receiver, Field
import math 
from robot_pop import * 
import random

"""
Main supervisor base 
Optimization algorithm - Collaboration-oriented 
Angel Sylvester 2022
"""
columns = 'agent id' + ',time step' + ',fitness' + ',xpos'+ ',ypos' + ',num col' + ',genotype' 

# global collected_count 
collected_count = []

# statistics collected 
overall_fitness_scores = []
initial_genotypes = []
pop_genotypes = [] 
total_found = 0
pairs = []
curr_best = -1 
population = []
ids = []

# set-up robot 
TIME_STEP = 32
robot = Supervisor()  # create Supervisor instance
timestep = int(robot.getBasicTimeStep())

# generalize id acquisition (will be same as robot assigned to), might change to someting different  

given_id = int(robot.getName()[11:-1]) - 1
    
#### allow personal supervisor to send important encounter info back to supervisor ## 
emitter = robot.getDevice("emitter")
emitter.setChannel(2)

#### receive info from supervisor regarding robot #####
receiver = robot.getDevice("receiver") 
receiver.enable(TIME_STEP)
receiver.setChannel(5) 

### allow personal supervisor to recieve encounter info from robot 
receiver_individual = robot.getDevice("receiver_processor") 
receiver_individual.enable(TIME_STEP)
receiver_individual.setChannel(int(given_id * 10)) # will be updated to be robot_id * 10 
emitter_individual = robot.getDevice("emitter")
emitter_individual.setChannel((int(given_id) * 10) - 1)

updated = False
fit_update = False 
start = 0 

prev_msg = ""
random.seed(10)
curr_fitness = 0
child = ""
coordination_type = ""
curr_robot_index = "" # should hopefully be overwritten if done correctly 

def generate_heading(assigned_id = None):
    global curr_robot_index
    global population
    
    curr_pos = [population[curr_robot_index].getPosition()[0], population[curr_robot_index].getPosition()[1]]
    
    if assigned_id == None: 
        closest_neigh = " "
        
        for i in range(len(population)):
            if (i != curr_robot_index): 
                other_pos = [population[i].getPosition()[0], population[i].getPosition()[1]]
                dis = math.dist(curr_pos, other_pos)
                if closest_neigh == " ":
                    closest_neigh = str(population[i].getId())
                    curr_dist = dis
                    other_index = i
                elif dis < curr_dist: 
                    closest_neigh = str(population[i].getId())
                    curr_dist = dis 
                    other_index = i
                    
        neigh_closest = other_index 
        
    else: 
        neigh_closest = assigned_id 
    
    print(f'following agent {neigh_closest} that is closest to {curr_robot_index}')
    curr_x, curr_y = population[curr_robot_index].getPosition()[0], population[curr_robot_index].getPosition()[1]
    neigh_x, neigh_y = population[neigh_closest].getPosition()[0], population[neigh_closest].getPosition()[1]
    updated_heading = round(math.atan2(neigh_y-curr_y,neigh_x-curr_x),2)
    
    print(f'heading for {curr_robot_index} should be {updated_heading}')
    
    
    
# based off given id of robot assigned 
def find_nearest_robot_genotype(r_index):
    global population 
    global reproduce_list 
    global collected_count 
    global pairs 
    global overall_fitness_scores
    global prev_msg 

    closest_neigh = " "
    curr_robot = population[r_index]
    curr_dist = 1000 # arbitrary value 
    curr_fitness = overall_fitness_scores[r_index]
    # print('overall fitness list', overall_fitness_scores)
    curr_overall_fitness = overall_fitness_scores[r_index]
    other_fitness = 0
    
    curr_pos = [curr_robot.getPosition()[0], curr_robot.getPosition()[1]]
    other_index = r_index
    
    for i in range(len(population)):
        if (i != r_index): 
            other_pos = [population[i].getPosition()[0], population[i].getPosition()[1]]
            dis = math.dist(curr_pos, other_pos)
            if closest_neigh == " ":
                closest_neigh = str(population[i].getId())
                curr_dist = dis
                other_fitness = overall_fitness_scores[i]
                other_index = i
            elif dis < curr_dist: 
                closest_neigh = str(population[i].getId())
                curr_dist = dis 
                other_fitness = overall_fitness_scores[i]
                other_index = i
                
    return other_index


def message_listener(time_step):
    global total_found 
    global collected_count 
    global found_list
    global pop_genotypes
    global population
    global curr_size
    global pairs 
    global overall_fitness_scores
    global curr_best
    global child 
    global comparing_genes
    global ids 
    global curr_robot_index

    if receiver.getQueueLength()>0:
        message = receiver.getData().decode('utf-8')
            
        if 'fitness-scores' in message:
            fs = message.split(" ")[1:]
            overall_fitness_scores = [int(i) for i in fs]
            receiver.nextPacket()
         
        ## resets to current population genotypes     
        elif 'generation-complete' in message:
            curr_best = -1 
            pop_genotypes = message.split(" ")[1:]
            overall_fitness_scores = [0 for i in range(len(pop_genotypes))]
            
            # initial child
            if not comparing_genes: 
                child = reproduce(pop_genotypes[int(given_id)], pop_genotypes[int(given_id)])
                child = "child" + str(child) 
                emitter_individual.send(child.encode('utf-8'))
            else: 
                child_1, child_2 = reproduce(pop_genotypes[int(given_id)], pop_genotypes[int(given_id)], multi = comparing_genes)
                child = "child-" + str(child_1) + '-' + str(child_2) 
                emitter_individual.send(child.encode('utf-8'))
            
            receiver.nextPacket()
        
        ## access to robot id for node acquisition    
        elif 'ids' in message: 
            id_msg = message.split(" ")[1:]
            population = []
            
            for id in id_msg: # will convert to nodes to eventual calculation 
                node = robot.getFromId(int(id))
                population.append(node)
                ids.append(id)
                
                
            receiver.nextPacket() 
            
        elif 'size' in message: 
            population = []
            size = int(message[4:]) 
            receiver.nextPacket()
            
        elif 'comparing' in message: 
            if message.split('-')[1] == 'False':
                comparing_genes = False
            else: 
                comparing_genes = True 
            
            receiver.nextPacket()
            
            
        else: 
            receiver.nextPacket()
            
    if receiver_individual.getQueueLength()>0:  
        message_individual = receiver_individual.getData().decode('utf-8')
        # print('indiviudal msgs --', message_individual)
            
        if 'encounter' in message_individual: 
            robo_index = int(message_individual.split('-')[0])
            # reproduce_list.append(robo_index) 
            # print('robot found -- checking genotype', robo_index) 
            curr_orient = message_individual.split('[')[-1]
            
            # only store best genotype 
            other_index = find_nearest_robot_genotype(robo_index)
            if overall_fitness_scores[other_index] > curr_best: 
                if not comparing_genes: 
                    curr_best = other_index
                    child = 'child' + str(reproduce(pop_genotypes[robo_index], pop_genotypes[curr_best]))
                    # print('child ---', child) 
                    # uncomment if want to just use curr_best genotype 
                    # child = 'child' + str(pop_genotypes[curr_best]))
                    
                    emitter_individual.send(child.encode('utf-8'))
                else: 
                    child_1, child_2 = reproduce(pop_genotypes[int(given_id)], pop_genotypes[int(curr_best)], multi = comparing_genes)
                    child = "child-" + str(child_1) + '-' + str(child_2) 
                    emitter_individual.send(child.encode('utf-8'))
                    
            else: 
                child = 'child' + str(reproduce(pop_genotypes[robo_index], pop_genotypes[robo_index]))
                emitter_individual.send(child.encode('utf-8'))
                # emitter_individual.send('penalize'.encode('utf-8'))
                   
            comm_information = "comm-" + str(robo_index) + "-" + str(other_index) + "-[" + str(curr_orient)
            emitter_individual.send(comm_information.encode('utf-8'))
            
            receiver_individual.nextPacket()
            
        if 'assigned' in message_individual:
            curr_robot_index = int(message_individual.split('-')[-1])
            generate_heading() # TODO: remove, just for testing purposes
            
            
            receiver_individual.nextPacket() 
           
            
        else: 
            receiver_individual.nextPacket()
     
    
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
    global timestep 
    
    while robot.step(timestep) != -1: 
        message_listener(timestep)        
     
  
def main(): 
    run_optimization()
         
main()
                    
            
            
