from controller import Supervisor, Node, Keyboard, Emitter, Receiver
import statistics 
import pandas as pd

"""
Main supervisor base 
Optimization algorithm - Collaboration-oriented 
Angel Sylvester 2022
"""

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance

# get info from this def 
khepera_node = robot.getFromDef("khepera")

# emitter to send info to robots 
emitter = robot.getDevice("emitter")
emitter.setChannel(1)

# set receiver to receive info 
receiver = robot.getDevice("receiver") 
receiver.enable(TIME_STEP)
receiver.setChannel(2) 

num_generations = 50
population = [khepera_node]

def restore_positions():
    pass 

# runs simulation for designated amount of time 
def run_seconds(t,reset_position=False):
    n = TIME_STEP / 1000*32 # convert ms to s 
    start = robot.getTime()
    
    
    
    while robot.step(TIME_STEP) != -1:
        # run robot simulation for 30 seconds (if t = 30)
        increments = TIME_STEP / 1000
        if robot.getTime() - start > t: 
            break 
        if reset_position: 
            restore_positions()
            
        # assuming only getting messages about removing nodes 
        if receiver.getQueueLength()>0:
            message = receiver.getData().decode('utf-8')
            if message == 'delete': # ideally would have name of node getting deleted
                receiver.nextPacket()
                object = robot.getFromDef("RED_STICK")
                # object.remove()
            
    return 
            
def reproduce(robot_node): 
    # update parameters to hopefully improve performance 
    # potential changes: speed
    return 
    
def send_genotype(r_node_genotype):
    genotype_string = [str(g) for g in r_node_genotype]
    genotype_string = ','.join(genotype_string)
    
    emitter.send(genotype_string.encode('utf-8'))
    

# fitness function for individual robot 
def eval_fitness(r_node_genotype):
    send_genotype(r_node_genotype)
    run_seconds(60)
    emitter.send('return_fitness'.encode('utf-8'))
    fitness = 0 
    
    while robot.step(TIME_STEP) != -1: 
        if receiver.getQueueLength() > 0: 
            receiver.getData().decode('utf-8')
            receiver.nextPacket()
            r_fit = float(message)
               
    return fitness # will eventually return fitness calculation  
    
def run_optimization():
    for gen in range(num_generations): 
        pop_fitness = [] 
        
        for robot in population: 
            # retrieves info about robots 
            geno_fit = eval_fitness('genotype') # placeholder for now 
            pop_fitness.append(geno_fit)
            
        mean_fit = statistics.mean(pop_fitness)
        for robot in population: 
            geno_fit = eval_fitness('genotype') # must find way to connect each robot 
            if geno_fit < mean_fit: 
                reproduce(robot)
                
    return 
        
            
def main(): 
    translation_field = khepera_node.getField('translation')
    rotation_field = khepera_node.getField('rotation')
    
    # run_optimization()

    restore_positions()
  
    for gen in range(num_generations): 
        translation_field.setSFVec3f([0,0,0]) # reset robot position
        rotation_field.setSFRotation([0, 0, 1, 0])
        khepera_node.resetPhysics()
        
        run_seconds(20) 
        print('new generation starting')

        pass  

         
main()
                    
            
            