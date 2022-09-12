# parameters of LAS:
# incorporates number of robots m

import numpy as np

class LAS():
    def __init__(self, top_right, bot_left, num_cells, curr_pos): # assuming 3 rows, arbitrarily
        self.trx, self.try = top_right
        self.blx, self.bly = bot_left
        self.num_cells = num_cells
        self.prob_vector = []
        self.neighbors = {}
        self.cells = [] # represents ranges for designated # of cells, right top x,y and bot x, y val
        self.curr_tile = self.locate_cell(curr_pos)

        # create a vector with equal probability of locating target in all cells
        for i in range(num_cells): # current cell
            curr_i_vector = []
            for j in range(num_cells)
                curr_i_vector.append(1/num_cells)
            self.prob_vector.append(curr_i_vector)

        num_per_row = num_cells // 3

        for i in range(3):
            for j in range(num_per_row):
                top_rightx, top_righty = self.trx*j, self.try*i
                bot_rightx, bot_righty = self.blx*j, self.bly*i
                self.cells.append([top_rightx, top_righty, bot_rightx, bot_righty])

        # precalculates neighbors
        for cell in self.cells:
            neigh = []
            for other_cell in self.cells:
                if cell == other_cell:
                    continue
                else:
                    tpx, tpy, brx, bry = cell
                    otpx, otpy, obrx, obry = other_cell
                    if (abs(tpx - otpx) == self.trx and abs(tpy - otpy) == self.try):
                        neigh.append(other_cell)
                self.neighbors[cell] = neigh


    def update(self, original_tile, curr_pos):
        neighbors = self.neighbors[original_tile]
        x, y = curr_pos
        curr_closest = neighbors[0] 
        otpx, otpy, obrx, obry = original_tile
        
        # check if still in current tile 
        if x < otpx and x > obrx and y > obry and y < otpy: 
            return self.neighbors.index(original_tile)
        
        for n in neighbors: 
            # check if in neighboring tiles 
            tpx, tpy, brx, bry = n 
            
            if x < tpx and x > brx and y > bry and y < tpy: 
                return self.neighbors.index(original_tile)
        
        return 'error update not possible'
               

    def locate_cell(self, curr_location):
        x, y = curr_location
        for c in self.cells: 
            otpx, otpy, obrx, obry = cell 
            
            if x < otpx and x > obrx and y > obry and y < otpy: 
                return self.cells.index(cell)  
            
        return 'locate cell error'
