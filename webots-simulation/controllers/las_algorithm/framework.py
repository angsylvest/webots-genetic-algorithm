# parameters of LAS:
# incorporates number of robots m

import numpy as np

class LAS():
    def __init__(self, top_right, bot_left, num_cells): # assuming 3 rows, arbitrarily
        self.trx, self.try = top_right
        self.blx, self.bly = bot_left
        self.num_cells = num_cells
        self.prob_vector = []
        self.neighbors = {}
        self.cells = [] # represents ranges for designated # of cells, right top x,y and bot x, y val

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


    def update(self):
        pass

    def locate_cell(self, curr_location):
        pass

    def find_neightbors(curr_bearing):
        pass
