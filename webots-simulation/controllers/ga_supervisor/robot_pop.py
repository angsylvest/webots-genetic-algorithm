# Generates ruleset for each robot 

import random 
import pandas as pd 
import numpy as np 

def create_initial_genotype():
    return "1101100011*1000000011100110000001100000110100111101001111100110111000101111110010001101000001100000111111101000101010101011000111100000101111000010001001111010010111011010001101110110010100001000110010000111110101001110110010101001010100011011100001110110100000111001111101001100110101000011001101010011110000101100110011010100100101100101101110010111111100001010110101000010001010110111100000010010010111101011100001000100010100100001100110100011111101111010000111010101100110110100111001101110011010111000110001010110101000010000111001100101110010010000001000110001011010111111000100100001111100111011101101011110100000111111001010011100101101110001000010001111010110010011111101101001011100100111111110101001000110001111000001011100110101111110001000100110011110010110001011100010100101101100110001101110111000111000110001000000110100111011001001100110010010000011001000010101001111111110111110110110100100000010010000011010011101011011010011011001111101010100000000010000001011011111110111001111101110011100*1010111010010001010111101011111101100100110111100010011101010010100111111010010111001101101001111101110100111001100101010100000100100001001001000001001001001100111011000010001110101101001001000001000110000011111100001111011000000101001101010011100010001100010010110111010110100110111100100101000000010001000010111011100111110111000001000110000101001010011010001000011110111000101011001010110110010010110001110110100011010000111011101100010001001001100100111001101001010000101010101000110000011111010011001100011111110101101011000010101010100101000110*"

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
    new_genotype = ""
    mom = r1.split("*")
    dad = r2.split("*")
        
    for i in range(len(mom)-1): # assuming this is a list of genotypes 
        
        child = crossover(mom[i], dad[i]) 
        child = mutate(child, 0.2) + "*"
        new_genotype += child 
    
    return new_genotype 
    
def crossover(m, d):
    new_child = ""
    
    random_start = random.randint(0,len(m)-1)
    
    new_child += m[:random_start] + d[random_start:] 
    
    return new_child 

def mutate(c, mut_prob): 

    size = len(c) 
    for i in range(size): 
        if random.random() < mut_prob: 
            if i == 0: 
                # choose random position 
                p = random.randint(0,len(c)/2)
                # switch to different val 
                if c[p] == str(1):
                    c = c[:p] + "0" + c[p+1:]
                else: 
                    c = c[:p] + "1" + c[p+1:]
            
            
            elif i == 1: 
                # choose random position 
                p = random.randint(len(c)/2, len(c)-1)
                # switch to different val 
                if c[p] == str(1):
                    c = c[:p] + "0" + c[p+1:] 
                else: 
                    c = c[:p] + "1" + c[p+1:] 
            
            else: 
                pass 
            
    return c  
    

    

