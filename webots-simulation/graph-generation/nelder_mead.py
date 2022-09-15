# nelder mead multivariate soptimization to find max (placeholder, untested) 

from scipy.optimize import minimize
from numpy.random import rand
from numpy.random import randn

def objective(x):
    # will return multi-variate poisson function 
    pass 
    
# define range for input
r_min, r_max = 0, 1
# define the starting point as a random sample from the domain
pt = r_min + rand(1) * (r_max - r_min)
# perform the search
result = minimize(objective, pt, method='nelder-mead')
# summarize the result
print('Status : %s' % result['message'])
print('Total Evaluations: %d' % result['nfev'])
# evaluate solution
solution = result['x']
evaluation = objective(solution)
print('Solution: f(%s) = %.5f' % (solution, evaluation))  
