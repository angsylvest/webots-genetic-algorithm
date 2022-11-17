# parameters of LAS:
# incorporates number of robots m

# import numpy as np
import math 
import random

# (1, 1) (-1, -1)
class LAS():
    def __init__(self, top_right = (-1, 1), bot_left = (1, -1), num_cells = 9, curr_pos = (0,0)): # assuming 3 rows, arbitrarily
        self.trx, self.tr_y = top_right
        self.blx, self.bly = bot_left
        self.num_cells = num_cells
        self.prob_vector = []
        self.neighbors = {}
        self.cells = [] # represents ranges for designated # of cells, right top x,y and bot x, y val
        self.iterations_threshold = 50

        # create a vector with equal probability of locating target in all cells
        for i in range(num_cells): # current cell
            curr_i_vector = []
            for j in range(num_cells):
                curr_i_vector.append(1/num_cells)
            self.prob_vector.append(curr_i_vector)

        num_per_row = num_cells // 3
        
        # row increment 
        self.r_incre = abs((self.blx - self.trx)) / num_per_row 
        
        # column increment 
        self.c_incre = abs(self.tr_y - self.bly) / 3
        # need to specy num to increment by, so equally distributed 
        for i in range(3): # arbitrary 3 rows 
            for j in range(num_per_row):
                top_rightx, top_righty = self.trx + (self.r_incre*j), self.tr_y - (self.c_incre*i)
                bot_rightx, bot_righty = top_rightx + self.r_incre, top_righty - self.c_incre
                self.cells.append((top_rightx, top_righty, bot_rightx, bot_righty))
                
        # self.prob_vector = np.full(len(self.cells), (1/(len(self.cells)))).tolist()
        self.prob_vector = [(1/(len(self.cells))) for i in range(len(self.cells))]
        # print('len of prob', len(self.prob_vector))
        # self.M_vector = np.full(len(self.cells), 0).tolist()
        self.M_vector = []
        # print(self.M_vector)
        self.dir_vector = 0 # direction that robot should persist towards to reach tile 

        self.curr_tile = self.locate_cell(curr_pos)
        self.target = self.curr_tile
        
        # precalculates neighbors
        for cell in self.cells:
            neigh = []
            for other_cell in self.cells:
                if cell == other_cell:
                    continue
                else:
                    tpx, tpy, brx, bry = cell
                    otpx, otpy, obrx, obry = other_cell
                    
                    # print('cell dims', cell)
                    # print('other cell dims', other_cell)
                    
                    if (abs(tpx - otpx) <= abs(self.r_incre) and abs(tpy - otpy) <= abs(self.c_incre)):
                        neigh.append(other_cell)
            self.neighbors[str(cell)] = neigh
            
        self.res_factor = 70 
        self.delta_pen = 1 / ((len(self.cells)) * self.res_factor)
        # print(self.cells)
                        

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
        
    def reward(self, curr_tile):
    
        if curr_tile in self.M_vector:
            return self.prob_vector
        
        for p in range(len(self.prob_vector)): 
            self.M_vector.append(curr_tile)
            sum = 1 
            if p != curr_tile: 
                new_p = p - self.delta_pen
                if new_p < 0:
                    new_p = 0
                
                self.prob_vector[p] = new_p
                sum += new_p 
                
            else: 
                self.prob_vector[p] = 1 - sum 
                
        return self.prob_vector 
        
    def penalize(self, curr_tile): 
        curr_min = self.prob_vector[0]
        curr_max = self.prob_vector[0]
    
        for p in range(len(self.prob_vector)): 
            sum = 1 
            if p != curr_tile: 
            
                if self.prob_vector[p] > self.delta_pen * (len(self.cells) - 1):
                    in_min = self.delta_pen * (len(self.cells) - 1)
                else: 
                    in_min = self.prob_vector[p]
                    
                
                if self.prob_vector[p] + (in_min / (len(self.cells) - 1)) > 1: 
                    self.prob_vector[p] = self.prob_vector[p] + (in_min / (len(self.cells) - 1))
                else: 
                    self.prob_vector[p] = 1
                    
            else: 
                if self.prob_vector[p] - (len(self.cells) - 1) * self.delta_pen > 0: 
                    self.prob_vector[p] = self.prob_vector[p] - (len(self.cells) - 1) * self.delta_pen
                else: 
                    self.prob_vector[p] = 0
                    
                
        return self.prob_vector 
        
    def re_gather(self, curr_tile): 
        new_cell = random.choices(self.cells, self.prob_vector)[0]
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
        

    def re_direct(self, curr_tile): 
        # new_cell = self.target
        # print('curr cell --', curr_tile)
        # print('target cell --', self.target)
        # new_tile = self.cells[new_cell]
        new_tile = self.target
        new_cell = self.cells[new_tile]

        if curr_tile == new_tile: 
            return round(random.choice([0,0, math.pi/2, -math.pi/2]),2)
         
        tpx, tpy, brx, bry = self.cells[curr_tile]
        mx, my = (tpx + brx)/2, (tpy + bry)/2 # calculating midpoint 
        # print(mx, my)
        otpx, otpy, obrx, obry = self.cells[new_tile]
        omx, omy = (otpx + obrx)/2, (otpy + obry)/2 # calculating midpoint 
        # print(omx, omy)
        direction = math.atan2((omy - my),(omx - mx))
        self.target = new_tile
        self.dir_vector = direction 
         
        return round(direction, 2)
