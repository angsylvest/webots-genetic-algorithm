# would do bayesian updating approach to identify proper rule sets on the fly 
# from utils.bayes import NArmedBanditDrift

import math 

class CompleteMap():
    def __init__(self, obstacle_locations, dims, obstacle_size, agent_size, x_bounds, y_bounds):
        self.obstacle_locations = obstacle_locations
        self.agent_positions = []
        self.agents = {}
        self.dims = dims
        self.obstacle_size = obstacle_size
        self.agent_size = agent_size
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds

    def update_agent_positions(self, agents):
        self.agents = agents

        for key in self.agents: 
            self.agent_positions.append(self.agents[key])

    def subset(self, dim_size, central_loc):
        min_x = central_loc[0] - dim_size/2
        max_x = central_loc[0] + dim_size/2
        min_y = central_loc[1] - dim_size/2
        max_y = central_loc[1] + dim_size/2

        subset_obstacles = [(x, y) for (x, y) in self.obstacle_locations if min_x <= x <= max_x and min_y <= y <= max_y]
        # subset_agents = [(x, y) for (x, y) in self.agent_positions if min_x <= x <= max_x and min_y <= y <= max_y]
        subset_agents = {key: value for key, value in self.agents.items() if min_x <= value[0] <= max_x and min_y <= value[1] <= max_y}

        # print(f'subsetted agents: {subset_agents}')
        return subset_obstacles, subset_agents
    
    def find_central(self, curr_pose):
        pass

    
    
class LocalMap():
    def __init__(self, obstacle_pos, obstacle_size, agent_pos, agent_size, local_dim, x_bounds, y_bounds, central_loc):
        self.obstacle_pos = obstacle_pos
        self.obstacle_size = obstacle_size
        self.agent_pos = agent_pos 
        self.agent_size = agent_size
        self.local_dims = local_dim
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.central_loc = central_loc

        self.num_envs = 4
        self.duration = 2 

        # self.bayes = [[1, 1, 1, 1] for i in range(local_dim*local_dim)]
        # self.bayes = [NArmedBanditDrift for n in range(num_envs)]

    def update_agent_pos(self, agent_list): 
        self.agent_pos = agent_list
    
    # for queue coordination strat
    def queue_times(self): 
        queue_assignments = {}
        for i in range(len(self.agent_pos)): 
            key_name = f'agent-{i}'
            queue_assignments[key_name] = self.duration*i

        return queue_assignments
    
    def is_dense_enough(self):
        return True
    
    def process_output(self, curr_strat_index):
        if curr_strat_index == 0:
            return self.assign_leaders()
        elif curr_strat_index == 1: 
            return self.queue_times()
        elif curr_strat_index == 2: 
            return self.generate_dispersion_vectors()
        else: 
            print('whoops does not exist')



    # for flocking strategy 
    def assign_leaders(self):
        # can be updated each time to be slightly before where agent is
        assignments = {}
        leader = self.agent_pos['agent-0'] # just random 
        assignments['agent-0'] = "leader"

        for i in range(1, len(self.agent_pos)):
            key_name = f'agent-{i}'
            curr_agent = self.agent_pos[key_name]
            assignments[key_name] = self.get_updated_goal(curr_agent, leader, 0.2)

        return assignments
    

    # for dispersal strategy 
    def generate_dispersion_vectors(self):
        agent_vectors = {}
        center = self.calculate_center(self.agent_pos)

        for i in range(len(self.agent_pos)):
            key_name = f'agent-{i}'
            # print(f'self.agent_pos[i]: {self.agent_pos[key_name]} and center: {center}')
            orient = self.calculate_orientation(self.agent_pos[key_name], center)
            agent_vectors[key_name] = orient
             
        return agent_vectors 


    # other relevant functions for each strategy 
    def get_updated_goal(self, current_pos, goal_pos, distance): 
        direction = (goal_pos[0] - current_pos[0], goal_pos[1] - current_pos[1])
        distance_to_goal = math.sqrt(direction[0]**2 + direction[1]**2)
        if distance_to_goal > 0:
            direction = (direction[0] / distance_to_goal, direction[1] / distance_to_goal)
        new_pos = (round(current_pos[0] - direction[0] * distance,2), round(current_pos[1] - direction[1] * distance),2)
        return new_pos


    def calc_total_area(self):
        min_x = max(self.x_bounds[0], self.central_loc[0] - self.local_dims/2)
        max_x = min(self.x_bounds[1], self.central_loc[0] + self.local_dims/2)
        min_y = max(self.y_bounds[0], self.central_loc[1] - self.local_dims/2)
        max_y = min(self.y_bounds[1], self.central_loc[1] + self.local_dims/2)
        total_area = (max_x - min_x) * (max_y - min_y)

        return total_area


    def calculate_area_within_bounds(self, center, dims):
        # Calculate the bounds of the rectangle within the local map
        min_x = self.x_bounds[0] # max(self.x_bounds[0], center[0] - dims[0]/2)
        max_x = min(self.x_bounds[1], center[0] + dims[0]/2)
        min_y = self.y_bounds[0] # max(self.y_bounds[0], center[1] - dims[1]/2)
        max_y = min(self.y_bounds[1], center[1] + dims[1]/2)

        # Calculate the area within the bounds
        area = (max_x - min_x) * (max_y - min_y)
        
        # print(f'areas: {area}')
        return area


    def identify_obstacle_distr_type(self):
        # dense vs sparse based on # of obstacles (more than 50% vs less than 50% of open spaces)
        # also, type: (organized, organized (unpredictable), corridor, open)
        total_area = self.calc_total_area()
        
        # Initialize the total covered area to 0
        total_covered_area = 0
        
        # Calculate total area covered by obstacles
        for center in self.obstacle_pos:
            area = self.calculate_area_within_bounds(center, (self.obstacle_size, self.obstacle_size))
            total_covered_area += area
        
        # Calculate open space area by subtracting the total covered area from the total area
        open_space_area = total_area - total_covered_area
        obstacle_density = total_covered_area / total_area

        # print(f'total area: {total_area}')
        # print(f'total covered area: {total_covered_area}')
        # print(f'open_space area: {open_space_area}')
        # print(f'obstacle density: {obstacle_density}')

        return obstacle_density

    def identify_agent_distr_type(self):
        # dense vs sparse based on # of agents (more than 50% vs less than 50% of open spaces)
        total_area = self.calc_total_area()
        # Calculate total area covered by agents
        agent_area = sum(self.calculate_area_within_bounds(center, (self.agent_size, self.agent_size)) for center in self.agent_pos)
        open_space_area = total_area - agent_area
        agent_density = agent_area / total_area

        return agent_density 


    def bayes_sample(self):
        pass


    def calculate_center(self, positions):
        # positions is a dict so must convert
        total_x = 0
        total_y = 0
        count = len(positions)


        for position in positions:
            total_x += positions[position][0]
            total_y += positions[position][1]

        center_x = total_x / count
        center_y = total_y / count

        return center_x, center_y

    def normalize_vector(self, vec):
        magnitude = math.sqrt(sum(x**2 for x in vec))
        return tuple(x / magnitude for x in vec)

    def calculate_orientation(self, position, center):
        # Calculate the vector from center to position
        vec_to_position = (position[0] - center[0], position[1] - center[1])
        # Calculate the angle in radians
        angle_radians = round(math.atan2(vec_to_position[1], vec_to_position[0]),2)
        return angle_radians
    
    # communication-relevant functions 
    def choose_coordination_type(self):
        # rule set: (righthand rule, queue-based (pause and go), go-away, follow)
        # have way of labeling agent based on relative pos so that they know queue # 
        queue_set = {}
        
        return queue_set

    def find_local_neighbors(self):
        pass 

    # will include schedule so won't need constant flags .. 
    def share_rule_set(self):
        # sent as emitter 
        msg = ""

        return msg 
    

# def __main__():
    # obstacle_locations = [(1, 1), (2, 2), (5, 5), (7, 7)]
    # complete_map = CompleteMap(obstacle_locations=obstacle_locations, dims=10, obstacle_size=1, agent_size=0.5, bounds = (0, 10))
    # complete_map.update_agent_positions([(3,3), (6,6)])
    # central_loc = (5, 5)
    # dim_size = 10
    # map_subset_obs, map_subset_ag = complete_map.subset(dim_size=dim_size, central_loc=central_loc)

    # local_map = LocalMap(obstacle_pos=map_subset_obs, obstacle_size=dim_size, agent_pos=map_subset_ag, agent_size= 0.5, local_dim=dim_size, x_bounds=complete_map.x_bounds, y_bounds=complete_map.y_bounds, central_loc = central_loc)
    # local_map.identify_agent_distr_type()
    # local_map.identify_obstacle_distr_type()

# if __name__ == "__main__":
    # __main__()


