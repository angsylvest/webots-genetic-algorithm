# Generates ruleset for each robot 

import random 
import pandas as pd 

def create_random_population(size, gene_size): 
    # will control threshold of detection & speed 
    # rule-set generated for each robot 
    
    ruleset = []
    for pop in range(size): 
        for gene in range(gene_size):
            pass
        
    return str(random.randint(3, 6)) + " " + str(random.randint(3, 6)) + " " + str(random.randint(3, 6)) 
    
def generate_random_act():
    pass
    
def reproduce(r1, r2): 
    # if pass a certain threshold, will increment/decrement randomly by 
    # smaller amount 
    
    # if below, resample again 
    
    pass
    
    

