# How can you represent the L-infinity norm by introducing new variables and inequalities?
# The L-infinity norm norm(x, "inf") is defined as the largest absolute value of the entries of the vector x.

import cvxpy as cp
import numpy as np


def norm_inf_impl():
    x = cp.Variable(2)
    prob1 = cp.Problem(
        cp.Minimize(cp.norm(x, "inf")),
        # NOTE you can modify this constraint to change the solution.
        [x == np.array([2, -4])]
    )
    prob1.solve()
    print("optimal objective value", prob1.value)
    print("optimal x", x.value)

    z = cp.Variable()
    prob2 = cp.Problem(
        cp.Minimize(z),
        [
            z >= x,
            x >= -z,
            # NOTE you can modify this constraint to change the solution.
            x == np.array([2, -4]),
        ]
    )
    prob2.solve()
    print("optimal objective value", prob2.value)
    print("optimal x", x.value)
    print("optimal z", z.value)


norm_inf_impl()
