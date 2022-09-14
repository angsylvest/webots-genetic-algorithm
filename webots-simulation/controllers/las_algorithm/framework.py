# parameters of LAS:
# incorporates number of robots m

import numpy as np

class LAS():
    def __init__(self, top_right = (-1, 0.8), bot_left = (0, 0), num_cells = 9, curr_pos = (0,0)): # assuming 3 rows, arbitrarily
        self.trx, self.tr_y = top_right
        self.blx, self.bly = bot_left
        self.num_cells = num_cells
        self.prob_vector = []
        self.neighbors = {}
        self.cells = [] # represents ranges for designated # of cells, right top x,y and bot x, y val
        self.curr_tile = self.locate_cell(curr_pos)

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
                bot_rightx, bot_righty = self.blx + (self.r_incre*j), self.bly - (self.c_incre*i)
                self.cells.append((top_rightx, top_righty, bot_rightx, bot_righty))

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
                        

    def update(self, original_tile, curr_pos):
        neighbors = self.neighbors[str(self.cells[original_tile])]
        x, y = curr_pos
        curr_closest = neighbors[0] 
        otpx, otpy, obrx, obry = self.cells[original_tile]
        
        # check if still in current tile 
        if x > otpx and x < obrx and y < obry and y > otpy: 
            return original_tile
        
        for n in neighbors: 
            # check if in neighboring tiles 
            tpx, tpy, brx, bry = n
            
            if x > tpx and x < brx and y < bry and y > tpy: 
                return self.neighbors.index(n)
        
        return self.locate_cell(curr_pos)
               

    def locate_cell(self, curr_location):
        x, y = curr_location
        for c in self.cells: 
            otpx, otpy, obrx, obry = c 
            if round(x,1) > round(otpx,1) and round(x,1) < round(obrx,1) and round(y,1) > round(obry,1) and round(y,1) < round(otpy,1):
                print('comparison satisfied') 
                return self.cells.index(c)     
        return 'locate cell error'
