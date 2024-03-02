
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
        return subset_obstacles
    
    
class LocalMap():
    def __init__(self, obstacle_pos, obstacle_size, agent_pos, agent_size, local_dim):
        self.obstacle_pos = obstacle_pos
        self.obstacle_size = obstacle_size
        self.agent_pos = agent_pos 
        self.agent_size = agent_size
        self.local_dims = local_dim

    def identify_obstacle_distr_type(self):
        # dense vs sparse based on # of obstacles (more than 50% vs less than 50% of open spaces)
        # also, type: (organized, organized (unpredictable), corridor, open) 

        pass 

    def identify_agent_distr_type(self):
        # dense vs sparse based on # of agents (more than 50% vs less than 50% of open spaces)

        pass 

    def choose_coordination_type(self):
        # rule set: (righthand rule, queue-based (pause and go), go-away, follow)
        pass


    def find_local_neighbors(self):
        pass 


    def share_rule_set(self):
        # sent as emitter 
        msg = ""

        return msg 

