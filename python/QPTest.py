from cvxopt import matrix, solvers
import numpy as np

# NOTE: Program is numpy compatible.

Q = 2*np.array([ [2, .5], [.5, 1] ])
Q = matrix(Q)
p = np.array([1.0, 1.0])
p = matrix(p)
G = np.array([[-1.0,0.0],[0.0,-1.0]])
G = matrix(G)
h = np.array([0.0,0.0])
h = matrix(h)
A = matrix([1.0, 1.0], (1,2))
b = np.array([1.0])
b = matrix(b)
sol=solvers.qp(Q, p, G, h)

'''
Q = matrix([ [2.65, 1.10], [1.10, 0.53] ])
p = matrix([0.0, 0.0])
G = matrix([[0.0885,0.0635],[0.0,0.0]])
h = matrix([0.0935,0.0])
A = matrix([0.0, 0.0], (1,2))
b = matrix(0.0)

sol=solvers.qp(Q, p, G, h)
'''
print(sol['x'])

