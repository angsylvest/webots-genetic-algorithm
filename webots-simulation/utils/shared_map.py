
class CompleteMap():
    def __init__(self, obstacle_locations, dims, obstacle_size, agent_size):
        self.obstacle_locations = obstacle_locations
        self.agent_positions = []
        self.dims = dims
        self.obstacle_size = obstacle_size
        self.agent_size = agent_size

    def update_agent_positions(self, agent_positions):
        self.agent_positions = agent_positions

    def subset(self, dim_size, central_loc):
        min_x = central_loc[0] - dim_size/2
        max_x = central_loc[0] + dim_size/2
        min_y = central_loc[1] - dim_size/2
        max_y = central_loc[1] + dim_size/2

        subset_obstacles = [(x, y) for (x, y) in self.obstacle_locations if min_x <= x <= max_x and min_y <= y <= max_y]
        subset_agents = [(x, y) for (x, y) in self.agent_positions if min_x <= x <= max_x and min_y <= y <= max_y]
        
        return subset_obstacles, subset_agents
    
    
class LocalMap():
    def __init__(self, obstacle_pos, obstacle_size, agent_pos, agent_size, local_dim):
        self.obstacle_pos = obstacle_pos
        self.obstacle_size = obstacle_size
        self.agent_pos = agent_pos 
        self.agent_size = agent_size
        self.local_dims = local_dim

    def identify_obstacle_distr_type(self):
        total_area = self.local_dims ** 2
        obstacle_area = len(self.obstacle_pos) * self.obstacle_size ** 2
        open_space_area = total_area - obstacle_area
        obstacle_density = obstacle_area / open_space_area

        print(f'obstacle density: {obstacle_density}')

        # if obstacle_density > 0.5:
        #     return "dense"
        # else:
        #     return "sparse"

    def identify_agent_distr_type(self):
        total_area = self.local_dims ** 2
        agent_area = len(self.agent_pos) * self.agent_size ** 2
        open_space_area = total_area - agent_area
        agent_density = agent_area / open_space_area

        print(f'agent density: {agent_density}')

        # if agent_density > 0.5:
        #     return "dense"
        # else:
        #     return "sparse"
        

    # def identify_obstacle_distr_type(self):
    #     # dense vs sparse based on # of obstacles (more than 50% vs less than 50% of open spaces)
    #     # also, type: (organized, organized (unpredictable), corridor, open) 

    #     pass 

    # def identify_agent_distr_type(self):
    #     # dense vs sparse based on # of agents (more than 50% vs less than 50% of open spaces)

    #     pass 

    def choose_coordination_type(self):
        # rule set: (righthand rule, queue-based (pause and go), go-away, follow)
        pass


    def find_local_neighbors(self):
        pass 


    def share_rule_set(self):
        # sent as emitter 
        msg = ""

        return msg 
    

def __main__():
    obstacle_locations = [(1, 1), (2, 2), (5, 5), (7, 7)]
    complete_map = CompleteMap(obstacle_locations=obstacle_locations, dims=10, obstacle_size=1, agent_size=0.5)
    complete_map.update_agent_positions([(3,3), (6,6)])
    central_loc = (5, 5)
    dim_size = 10
    map_subset_obs, map_subset_ag = complete_map.subset(dim_size=dim_size, central_loc=central_loc)

    local_map = LocalMap(obstacle_pos=map_subset_obs, obstacle_size=dim_size, agent_pos=map_subset_ag, agent_size= 0.5, local_dim=dim_size)
    local_map.identify_agent_distr_type()
    local_map.identify_obstacle_distr_type()

if __name__ == "__main__":
    __main__()


