# parameters of inverted ant
# incorporates number of robots m

# import numpy as np
import math 
import random

# (1, 1) (-1, -1)
class InvertedAnt():
    def __init__(self, top_right = (-1, 1), bot_left = (1, -1), num_cells = 9, curr_pos = (0,0)): # assuming 3 rows, arbitrarily
        self.trx, self.tr_y = top_right
        self.blx, self.bly = bot_left
        self.num_cells = num_cells
        self.curr_prob_vector = []
        self.neighbors = {}
        self.cells = [] # represents ranges for designated # of cells, right top x,y and bot x, y val
        # self.cell_names = [i in range(len(self.cells)] # for when we use random.choices with the corresponding prob distribution 
        self.cmax = 0.2
        self.cmin = 0.04
        self.ctotal = 0 
        
        # relevant sim variables 
        self.d = 1
        self.delta = 0.25
        self.beta = 0.01
        self.Q_vector = [] # previously visited cells 

        num_per_row = num_cells // 3
        # row increment 
        self.r_incre = abs((self.blx - self.trx)) / num_per_row 
        # column increment 
        self.c_incre = abs(self.tr_y - self.bly) / 3
        
        # creates cells 
        for i in range(3): # arbitrary 3 rows 
            for j in range(num_per_row):
                top_rightx, top_righty = self.trx + (self.r_incre*j), self.tr_y - (self.c_incre*i)
                bot_rightx, bot_righty = top_rightx + self.r_incre, top_righty - self.c_incre
                self.cells.append((top_rightx, top_righty, bot_rightx, bot_righty))
                
        # is set of previously visited cells 
        
        self.dir_vector = 0 # direction that robot should persist towards to reach tile 
        self.curr_tile = self.locate_cell(curr_pos)
        self.target = self.curr_tile
        self.cvector = [0 for i in range(len(self.cells))]
        self.phermoneVector = [0 for cell in range(len(self.cells))]

        # precalculates neighbors
        for cell in self.cells:
            neigh = []
            for other_cell in self.cells:
                if cell == other_cell:
                    continue
                else:
                    tpx, tpy, brx, bry = cell
                    otpx, otpy, obrx, obry = other_cell
                    if (abs(tpx - otpx) <= abs(self.r_incre) and abs(tpy - otpy) <= abs(self.c_incre)):
                        neigh.append(other_cell)
            self.neighbors[str(cell)] = neigh
            
        self.initializeCurrentProbVector(self.curr_tile) # without phermones considered 
            
 
    def initializeCurrentProbVector(self, curr_tile): # just takes neighbors into account 
        neighbors = self.neighbors[str(self.cells[int(curr_tile)])]
        nei_tiles = []
        curr_prob_vector = [0 for i in range(len(self.cells))]
        
        for n in neighbors: 
            nei_tiles.append(int(self.cells.index(n)))
            
        num_nei = len(nei_tiles) 
        
        for i in range(len(self.cells)):
            if self.cells.index(self.cells[i]) in nei_tiles: 
                curr_prob_vector[self.cells.index(self.cells[i])] = self.cmax
                self.cvector[self.cells.index(self.cells[i])] = self.cmax
                self.ctotal += self.cmax
            else: 
                curr_prob_vector[self.cells.index(self.cells[i])] = self.cmin
                self.cvector[self.cells.index(self.cells[i])] = self.cmin
                self.ctotal += self.cmin
                
        # curr_prob_vector = [i/self.ctotal for i in curr_prob_vector]
        
        self.curr_prob_vector = curr_prob_vector
    
        
    def update_Q(self, curr_tile):
        if curr_tile not in self.Q_vector:
            self.Q_vector.append(curr_tile) 
            self.addPhermone(curr_tile)
            
        elif len(self.Q_vector) == len(self.cells):
            self.Q_vector = []
         
   
    # will update probability distribution as well   
    def addPhermone(self, curr_tile): 
        self.phermoneVector[int(curr_tile)] += self.d
        neighbors = self.neighbors[str(self.cells[int(curr_tile)])]
        neighbor_phermones = []
        
        for n in neighbors: 
            tile = int(self.cells.index(n))
            self.phermoneVector[tile] += self.delta
            neighbor_phermones.append(self.phermoneVector[tile])
            
        max_phermone = max(neighbor_phermones)
        
        # min_phermone = min(neighbor_phermones)
        
           
        for n in neighbors: 
            tile = int(self.cells.index(n))
            if self.phermoneVector[tile] == max_phermone: 
                self.ctotal -= self.cvector[tile]
                self.cvector[tile] = self.cmin
                self.ctotal += self.cmin
            
            elif all(self.phermoneVector[tile] <= p for p in neighbor_phermones):
                self.ctotal -= self.cvector[tile]
                self.cvector[tile] = self.cmax
                self.ctotal += self.cmax
           
        for val in range(len(self.cvector)): 
            self.curr_prob_vector[val] = self.cvector[val]
                
   
    # will update probability distribution as well 
    def declinePhermone(self, curr_tile): # after 1 sec or (t/1000)*32 time_step 
    
        neighbors = self.neighbors[str(self.cells[int(curr_tile)])]
        for p in self.phermoneVector:
            if p != 0: 
                p -= self.beta
                
        neighbor_phermones = []
        
        for n in neighbors: 
            tile = int(self.cells.index(n))
            self.phermoneVector[tile] += self.delta
            neighbor_phermones.append(self.phermoneVector[tile])
            
        max_phermone = max(neighbor_phermones)
           
        for n in neighbors: 
            tile = int(self.cells.index(n))
            if self.phermoneVector[tile] == max_phermone: 
                self.ctotal -= self.cvector[tile]
                self.cvector[tile] = self.cmin
                self.ctotal += self.cmin
            
            elif all(self.phermoneVector[tile] <= p for p in neighbor_phermones):
                self.ctotal -= self.cvector[tile]
                self.cvector[tile] = self.cmax
                self.ctotal += self.cmax
                
        for val in range(len(self.cvector)): 
            self.curr_prob_vector[val] = self.cvector[val]
          
            
    def re_gather(self, curr_tile): 
        new_cell = random.choices(self.cells, self.curr_prob_vector)[0]
        # print(self.cells) 
        # print(new_cell)
        new_tile = self.cells.index(new_cell)
         
        if curr_tile != new_tile: 
            tpx, tpy, brx, bry = self.cells[curr_tile]
            mx, my = (tpx + brx)/2, (tpy + bry)/2 # calculating midpoint 
            otpx, otpy, obrx, obry = new_cell
            omx, omy = (otpx + obrx)/2, (otpy + obry)/2 # calculating midpoint 
             
            direction = math.atan2(omy - my,omx - mx)
            self.target = new_tile
            self.dir_vector = direction 
             
            return round(direction, 2)
        else: 
            self.re_gather(curr_tile)
                                         
    # methods to locate cell 
    def update(self, original_tile, curr_pos):
        # print('original tile', original_tile)
        neighbors = self.neighbors[str(self.cells[int(original_tile)])]
        x, y = curr_pos
        curr_closest = neighbors[0] 
        otpx, otpy, obrx, obry = self.cells[original_tile]
        
        # check if still in current tile 
        if x > otpx and x < obrx and y > obry and y < otpy: 
            return original_tile
        
        for n in neighbors: 
            # check if in neighboring tiles 
            tpx, tpy, brx, bry = n
            
            if x > tpx and x < brx and y < bry and y > tpy: 
                return self.cells.index(n)
        
        return self.locate_cell(curr_pos)
               

    def locate_cell(self, curr_location):
        x, y = curr_location
        for c in self.cells: 
            otpx, otpy, obrx, obry = c 
            if round(x,1) >= round(otpx,1) and round(x,1) <= round(obrx,1) and round(y,1) >= round(obry,1) and round(y,1) <= round(otpy,1):
                # print(self.cells.index(c) ) 
                return self.cells.index(c)  
        print('locate cell error ', 'cells: ',self.cells,'current_location: ', curr_location)   
        return 'locate cell error'
        
        
    def re_direct(self, curr_tile): 
    
        new_cell = self.target
        new_tile = self.cells[new_cell]
        
        if curr_tile == new_cell: 
            return round(random.choice([0,0, math.pi/2, -math.pi/2]),2)
         
        tpx, tpy, brx, bry = self.cells[curr_tile]
        mx, my = (tpx + brx)/2, (tpy + bry)/2 # calculating midpoint 
        # print(mx, my)
        otpx, otpy, obrx, obry = self.cells[new_cell]
        omx, omy = (otpx + obrx)/2, (otpy + obry)/2 # calculating midpoint 
        # print(omx, omy)
        direction = math.atan2((omy - my),(omx - mx))
        self.target = new_tile
        self.dir_vector = direction 
         
        return round(direction, 2)
