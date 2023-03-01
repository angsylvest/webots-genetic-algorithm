# Generates ruleset for each robot 

import random 
# import pandas as pd 
# import numpy as np 


def create_random_population(size, gene_list): 
    # will control threshold of detection & speed 
    # rule-set generated for each robot 
    
    ruleset = ''
    pop_set = ''
    for pop in range(size): 
        for gene in gene_list:
            g_count = int(gene.split(" ")[-1])
            ruleset += generate_random_act(g_count) + "*"
        pop_set += ruleset + " "
    return pop_set         
        
        
def create_individal_genotype(gene_list):
    ruleset = ''
    for gene in gene_list:
        g_count = int(gene.split(" ")[-1])
        ruleset += generate_random_act(g_count) + "*"
    return ruleset
     
# will generate binary encoding for corresponding feature 
def generate_random_act(length):

    list_binary = ''.join([str(random.randint(0,1)) for i in range(length)])
    # np_binary = np.random.randint(2, size = length)
    # list_binary = ''.join([",".join(item) for item in str(np_binary)])
    return list_binary
    
    
def reproduce(r1, r2, multi=False): 
    global other_genotype 
    # if pass a certain threshold, will increment/decrement randomly by 
    # smaller amount 
    
    # if below, resample again 
    new_genotype = ''
    other_genotype = ''
    mom = r1.split("*")
    dad = r2.split("*")
    
    for i in range(len(mom)-1): # assuming this is a list of genotypes 
        # if i == 0:
            # mom[i] = mom[i][1:]
            # dad[i] = dad[i][1:]
        
        child = crossover(mom[i], dad[i]) 
        child = mutate(child, 0.2) + "*"
        new_genotype += child 
     
    if not multi: 
        return new_genotype 
    else: 
        return new_genotype, other_genotype 
    
def crossover(m, d):
    global other_genotype 
    
    new_child = ''
    other_child = '' 
    
    random_start = random.randint(0,len(m)-1)
    new_child += m[:random_start] + d[random_start:] 
    
    other_child += m[random_start:] + d[:random_start] 
    other_genotype = mutate(other_child, 0.2) + "*"
    
    return new_child 

def mutate(c, mut_prob):
    # print('child c', c, len(c)) 

    size = len(c) 
    for i in range(size): 
        if random.random() < mut_prob: 
            if i == 0: 
                # choose random position 
                p = random.randint(0,int(len(c)/2))
                # switch to different val 
                if c[p] == str(1):
                    c = c[:p] + "0" + c[p+1:]
                else: 
                    c = c[:p] + "1" + c[p+1:]
            
            
            elif i == 1: 
                # choose random position 
                p = random.randint(int(len(c)/2), len(c)-1)
                # switch to different val 
                if c[p] == str(1):
                    c = c[:p] + "0" + c[p+1:] 
                else: 
                    c = c[:p] + "1" + c[p+1:] 
            
            else: 
                pass 
            
    return c  
    

    

