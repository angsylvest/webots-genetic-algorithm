import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

import numpy as np
from shared_map import CompleteMap

def visualize_subset(subset_obstacles, central_loc, dims, obstacle_locations, obstacle_size, agent_positions, agent_size):
    fig, ax = plt.subplots()
    ax.set_xlim(0, dims)
    ax.set_ylim(0, dims)
    
    # Plot central location
    ax.scatter(*central_loc, color='red', label='Central Location')
    
    # Plot obstacles as squares
    for (x, y) in obstacle_locations:
        ax.add_patch(Rectangle((x - obstacle_size/2, y - obstacle_size/2), obstacle_size, obstacle_size, color='black'))

    # Plot subset obstacles
    for (x, y) in subset_obstacles:
        ax.add_patch(Rectangle((x - obstacle_size/2, y - obstacle_size/2), obstacle_size, obstacle_size, color='blue'))

    # Generate a list of colors for agents
    colors = plt.cm.rainbow(np.linspace(0, 1, len(agent_positions)))

    # Plot agents as circles with different colors
    for i, (x, y) in enumerate(agent_positions):
        circle = plt.Circle((x, y), agent_size/2, color=colors[i], label='Agent')
        ax.add_artist(circle)

    ax.legend()
    plt.show()

def __main__():
    obstacle_locations = [(1, 1), (2, 2), (5, 5), (7, 7)]
    complete_map = CompleteMap(obstacle_locations=obstacle_locations, dims=10, obstacle_size=1, agent_size=0.5)
    complete_map.update_agent_positions([(3,3), (6,6)])
    central_loc = (5, 5)
    dim_size = 3
    subset_obstacles = complete_map.subset(dim_size=dim_size, central_loc=central_loc)
    
    # Pass all required arguments to visualize_subset
    visualize_subset(subset_obstacles, central_loc, 5, complete_map.obstacle_locations, complete_map.obstacle_size, complete_map.agent_positions, complete_map.agent_size)

if __name__ == "__main__":
    __main__()

