# Generates ruleset for each robot 

import random 
import pandas as pd 
import numpy as np 



def create_random_population(size, gene_list): 
    # will control threshold of detection & speed 
    # rule-set generated for each robot 
    
    ruleset = ""
    pop_set = ""
    for pop in range(size): 
        for gene in gene_list:
            g_count = int(gene.split(" ")[-1])
            ruleset += generate_random_act(g_count) + "*"
        pop_set += ruleset + " "
    return pop_set         
        
        
def create_individal_genotype(gene_list):
    ruleset = ""
    for gene in gene_list:
        g_count = int(gene.split(" ")[-1])
        ruleset += generate_random_act(g_count) + "*"
    return ruleset
     
# will generate binary encoding for corresponding feature 
def generate_random_act(length):
    np_binary = np.random.randint(2, size = length)
    list_binary = ''.join([",".join(item) for item in np_binary.astype(str)])
    return list_binary
    
    
def reproduce(r1, r2): 
    # if pass a certain threshold, will increment/decrement randomly by 
    # smaller amount 
    
    # if below, resample again 
    new_genotype = []
    
    for i in range(len(r1)): # assuming this is a list of genotypes 
        mom = r1[i]
        dad = r2[i]
        
        child = crossover(mom, dad) 
        child = mutate(child) + "*"
        new_genotype.append(child) 
    
    return new_genotype 
    
def crossover(m, d): 
    new_child = ""
    
    random_start = random.randint(len(m))
    
    new_child += m[:random_start] + d[random_start] 
    
    return new_child 

def mutate(c, mut_prob): 

    size = len(c) 
    for i in range(size): 
        if random.random() < mut_prob: 
            if i == 0: 
                # choose random position 
                p = random.randint(len(c)/2)
                # switch to different val 
                if c[p] == 1:
                    c[p] = 0
                else: 
                    c[p] = 1 
            
            
            elif i == 1: 
                # choose random position 
                p = random.randint(len(c)/2, len(c))
                # switch to different val 
                if c[p] == 1:
                    c[p] = 0
                else: 
                    c[p] = 1 
            
            else: 
                pass 
            
    return c  
    

    

