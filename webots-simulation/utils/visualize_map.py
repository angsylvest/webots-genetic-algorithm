import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

import numpy as np
from shared_map import CompleteMap, LocalMap

def visualize_subset(subset_obstacles, central_loc, dims, obstacle_size, agent_positions, agent_size, map_dims):
    fig, ax = plt.subplots()
    
    # Calculate the limits for the x and y axes based on the central location and subset dimension
    min_x = max(0, central_loc[0] - dims/2)
    max_x = min(central_loc[0] + dims/2, map_dims)  # Assuming 10 is the total dimension of the map
    min_y = max(0, central_loc[1] - dims/2)
    max_y = min(central_loc[1] + dims/2, map_dims)  # Assuming 10 is the total dimension of the map
    
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    
    # Plot central location
    ax.scatter(*central_loc, color='red', label='Central Location')
    
    # Plot subset obstacles
    for (x, y) in subset_obstacles:
        ax.add_patch(Rectangle((x - obstacle_size/2, y - obstacle_size/2), obstacle_size, obstacle_size, color='blue'))

    # Generate a list of colors for agents
    colors = plt.cm.rainbow(np.linspace(0, 1, len(agent_positions)))

    # Plot agents as circles with different colors
    i = 0 
    for pos in agent_positions: 
        (x,y) = agent_positions[pos]
        circle = plt.Circle((x, y), agent_size/2, color=colors[i], label='Agent')
        ax.add_artist(circle)
        i += 1 

    ax.legend()
    plt.show()


def __main__():
    obstacle_locations = [(1, 1), (2, 2), (5, 5), (7, 7)]
    x_bounds = (0, 10)
    y_bounds = (0, 10)
    complete_map = CompleteMap(obstacle_locations=obstacle_locations, dims=10, obstacle_size=1, agent_size=0.5, x_bounds=x_bounds, y_bounds=y_bounds)
    agent_list = {"agent-0": (3.0,3.0), "agent-1": (6.0,6.0)}
    complete_map.update_agent_positions(agent_list)
    central_loc = (5, 5)
    dim_size = 10 # dims of local env
    map_subset_obs, map_subset_ag = complete_map.subset(dim_size=dim_size, central_loc=central_loc)
    
    local_map = LocalMap(obstacle_pos=map_subset_obs, obstacle_size=complete_map.obstacle_size, agent_pos=map_subset_ag, agent_size= 0.5, local_dim=dim_size, x_bounds=complete_map.x_bounds, y_bounds=complete_map.y_bounds, central_loc=central_loc)
    # local_map.identify_agent_distr_type()
    local_map.identify_obstacle_distr_type()

    print(f'testing output of each coordination strategy')

    print(f'strategy for flocking: {local_map.assign_leaders()}')
    print(f'strategy for queue: {local_map.queue_times()}')
    print(f'strategy for dispersal: {local_map.generate_dispersion_vectors()}')

    # Pass all required arguments to visualize_subset
    visualize_subset(map_subset_obs, central_loc, 5, complete_map.obstacle_size, map_subset_ag, complete_map.agent_size, complete_map.dims)

if __name__ == "__main__":
    __main__()

