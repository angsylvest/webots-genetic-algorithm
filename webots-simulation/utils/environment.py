# creates generic environment 

import random, math

class Environment():

    def __init__(self, env_type = "random", seed = 11, num_blocks = 20):
        # initialize relevant vars 
        self.env_type = env_type
        self.seed = seed
        self.num_blocks = num_blocks

    def generate_blocks(self):
        if self.env_type == "random": 
            return self.random()

        elif self.env_type == "power law": 
            return self.power_law()
        
        elif self.env_type == 'single source':
            return self.single_source()

        else: 
            print("Invalid/unimplemmented environment")


    def power_law(self):
        b_pos_to_generate = []
        if self.seed == 11: 
            seed_file = open('../../graph-generation/seed-11-pl.csv', 'r') 
            list = seed_file.readlines()
            for pos in list: 
                res = [float(i) for i in pos.strip('][\n').split(', ')]
                b_pos_to_generate.append(res)
        
        elif self.seed == 15: 
            seed_file = open('../../graph-generation/seed-15-pl.csv', 'r') 
            list = seed_file.readlines()
            for pos in list: 
                res = [float(i) for i in pos.strip('][\n').split(', ')]
                b_pos_to_generate.append(res)

        else: # regenerate a new set based off seed value 
            random.seed(self.seed)
            x_min = 1
            alpha = 2.5
            centers = []
    
            for i in range(self.num_blocks): # will be number of clusters instead of disparate blocks 
                r = random.random()
                x_smp = round(x_min * (1 - r) ** (-1 / (alpha - 1)))
                print(x_smp)
                if x_smp > 5: 
                    x_smp = 5 
                    
                center = random.choices([[round(random.uniform(0.9, -0.9),2), round(random.uniform(0.3, 0.85),2), 0.02], [round(random.uniform(0.9, -0.9),2), round(random.uniform(-1, 0.23),2), 0.02]])[0]
                other = center
                if len(centers) != 0:
                    while all(math.dist(center, other) < 0.2 and math.dist(center, (0,0,0.02)) < 0.8 for other in centers): # generate center until appropriate distance away 
                        center = random.choices([[round(random.uniform(0.9, -0.9),2), round(random.uniform(0.3, 0.85),2), 0.02], [round(random.uniform(0.9, -0.9),2), round(random.uniform(-1, 0.23),2), 0.02]])[0]
                else: 
                    while math.dist(center, (0,0,0.02)) < 0.8: # generate center until appropriate distance away 
                        center = random.choices([[round(random.uniform(0.9, -0.9),2), round(random.uniform(0.3, 0.85),2), 0.02], [round(random.uniform(0.9, -0.9),2), round(random.uniform(-1, 0.23),2), 0.02]])[0]
                centers.append(center)
        
                x, y, z = center[0],center[1], center[2]
                x_smp -= 1
                pot = []
                new_c = [x+0.05, y, z]
                new_c1 = [x, y + 0.05, z]
                new_c2 = [x-0.05, y, z]
                new_c3 = [x, y - 0.05, z]
                pot.append(new_c)
                pot.append(new_c1)
                pot.append(new_c2)
                pot.append(new_c3)
                
                for i in range(x_smp): # will be clumped in same location
                    b_pos_to_generate.append(pot[i]) 

        return b_pos_to_generate

    def random(self): 
        b_pos_to_generate = []
        if self.seed == 11: 
            seed_file = open('../../graph-generation/seed-11-rn.csv', 'r') 
            list = seed_file.readlines()
            for pos in list: 
                res = [float(i) for i in pos.strip('][\n').split(', ')]
                b_pos_to_generate.append(res)
        
        elif self.seed == 15: 
            seed_file = open('../../graph-generation/seed-15.csv', 'r') 
            list = seed_file.readlines()
            for pos in list: 
                res = [float(i) for i in pos.strip('][\n').split(', ')]
                b_pos_to_generate.append(res)

        else: 
            for i in range(self.num_blocks // 2): 
                pose = [round(random.uniform(0.9, -0.9),2), round(random.uniform(0.3, 0.85),2), 0.02]
                b_pos_to_generate.append(pose)
            
            for i in range(self.num_blocks // 2): 
                pose = [round(random.uniform(0.9, -0.9),2), round(random.uniform(-1, 0.23),2), 0.02]
                b_pos_to_generate.append(pose)

        
        return b_pos_to_generate
    

    def single_source(self):
        random.seed(self.seed)
        b_pos_to_generate = []
        for i in range(self.num_blocks): 
            pose = [round(random.uniform(0.9, -0.9),2), round(random.uniform(0.3, 0.85),2), 0.02]
            b_pos_to_generate.append(pose)
        
        # for i in range(self.num_blocks // 2): 
        #     pose = [round(random.uniform(0.9, -0.9),2), round(random.uniform(-1, 0.23),2), 0.02]
        #     b_pos_to_generate.append(pose)
        return b_pos_to_generate