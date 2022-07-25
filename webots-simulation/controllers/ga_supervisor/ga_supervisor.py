from controller import Supervisor, Node, Keyboard, Emitter, Receiver
import statistics 
import pandas as pd
from robot_pop import * 

"""
Main supervisor base 
Optimization algorithm - Collaboration-oriented 
Angel Sylvester 2022
"""

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

def restore_positions():
    pass 
    

# runs simulation for designated amount of time 
def run_seconds(t,waiting=False):
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
            # assuming only getting messages about removing nodes 
            if receiver.getQueueLength()>0:
                message = receiver.getData().decode('utf-8')
                if 'k1-fitness' in message: 
                    k1_fitness = int(message[10:])
                    print('k1 fitness', k1_fitness)
                    
                    receiver.nextPacket()
                elif 'k2-fitness' in message: 
                    k2_fitness = int(message[10:])
                    print('k2 fitness', k2_fitness)
                    
                    receiver.nextPacket()
                elif 'k3-fitness' in message:
                    k3_fitness = int(message[10:])
                    print('k3 fitness', k3_fitness)
                    
                    receiver.nextPacket()
                    
                elif message[0] == "$": # handles deletion of objects when grabbed
                    message = int(message[1:])
                    receiver.nextPacket()
                    obj_node = robot.getFromId(message)
                    if obj_node is not None: 
                        obj_node.remove()
                
            
    return 
            
def reproduce(robot_node): 
    # update parameters to hopefully improve performance 
    # potential changes: speed
    return 
    
# def send_genotype(r_node_genotype):
    # genotype_string = [str(g) for g in r_node_genotype]
    # genotype_string = ','.join(genotype_string)
    
    # emitter.send(genotype_string.encode('utf-8'))
    

# fitness function for each individual robot 
def eval_fitness():
    # send_genotype()
    # run_seconds(60)
    
    # sent the same number of times each robot appears in sim
    emitter.send('return_fitness'.encode('utf-8'))
    # fitness = 0 
    
    
    # while robot.step(TIME_STEP) != -1: 
        # if receiver.getQueueLength() > 0: 
            # receiver.getData().decode('utf-8')
            # receiver.nextPacket()
            # r_fit = float(message)
               
    # return fitness # will eventually return fitness calculation  
    
def run_optimization():
    for gen in range(num_generations): 
        
        pop_fitness = [] 
        
        # send relevant genotypes to each robot, handler
        pop_genotypes = create_random_population(len(population), 1)
        emitter.send(str("#" + str(pop_genotypes)).encode('utf-8'))
        
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
                    
            
            