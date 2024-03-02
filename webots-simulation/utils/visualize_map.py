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
    for i, (x, y) in enumerate(agent_positions):
        circle = plt.Circle((x, y), agent_size/2, color=colors[i], label='Agent')
        ax.add_artist(circle)

    ax.legend()
    plt.show()


def __main__():
    obstacle_locations = [(1, 1), (2, 2), (5, 5), (7, 7)]
    x_bounds = (0, 10)
    y_bounds = (0, 10)
    complete_map = CompleteMap(obstacle_locations=obstacle_locations, dims=10, obstacle_size=1, agent_size=0.5, x_bounds=x_bounds, y_bounds=y_bounds)
    complete_map.update_agent_positions([(3,3), (6,6)])
    central_loc = (5, 5)
    dim_size = 10
    map_subset_obs, map_subset_ag = complete_map.subset(dim_size=dim_size, central_loc=central_loc)
    
    local_map = LocalMap(obstacle_pos=map_subset_obs, obstacle_size=complete_map.obstacle_size, agent_pos=map_subset_ag, agent_size= 0.5, local_dim=dim_size, x_bounds=complete_map.x_bounds, y_bounds=complete_map.y_bounds, central_loc=central_loc)
    # local_map.identify_agent_distr_type()
    local_map.identify_obstacle_distr_type()

    # Pass all required arguments to visualize_subset
    visualize_subset(map_subset_obs, central_loc, 5, complete_map.obstacle_size, map_subset_ag, complete_map.agent_size, complete_map.dims)

if __name__ == "__main__":
    __main__()

