"""
Integrates info collected from simulation into graphs 
"""

import pandas as pd 
import matplotlib.pyplot as plt 
import matplotlib.pylab as pl
import numpy as np 

def generate_fitness_csvs(list_pds):
    for item in list_pds:
        print(item)
    df = pd.concat(list_pds)
    df.to_csv('summary-fitness.csv')
    

def generate_fitness(csv_summary_fitness):

    x_list = []
    y_list = []
    
    df = pd.read_csv(csv_summary_fitness)
    
    # want to plot, separating each agent 
    labels = set(df['agent id'].values)
    colors = pl.cm.jet(np.linspace(0,1,len(labels)))
    for key, color in zip(labels, range(len(labels))):
        data_x = df.loc[df['agent id'] == key]["time step"]
        data_y = df.loc[df['agent id'] == key]["fitness"]
        plt.plot(data_x, data_y, color = colors[color], label = key)
      
    plt.title("Fitness Progression Over Time")
    plt.xlabel("time step")
    plt.ylabel("fitness")
    
    plt.legend()
    plt.show()
    plt.savefig("fitness-output.png")   
    
    print('generated fitness graph for single trial')  
    

def generate_paths():
    pass 
    
def combine_graphs_fitness():
    # will combine fitness csvs for non ga, and ga 
    df_ga = pd.read_csv()
    df_crw = pd.read_csv()
    
    
    
    pass 

