import cvxpy as cp
from cvxpygen import cpg

# model problem
x = cp. Variable(n, name='x')
G = cp. Parameter((m, n), name='G')
h = cp. Parameter(m, name='h')
problem = cp. Problem(cp. Minimize(cp. sum_squares(G@x - h)), [x >= 0])
# generate code
cpg. generate_code(problem)
