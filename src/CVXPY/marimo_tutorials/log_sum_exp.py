# Challenge problem 2
#
# How can you represent log_sum_exp by introducing new variables and constraints?
# Hint: use cp.ExpCone(t, y, z), which mathematically represents y > 0, y*exp(t/y) <= z

import cvxpy as cp
import numpy as np


def log_sum_exp_impl():
    x = cp.Variable(2)
    prob1 = cp.Problem(
        cp.Minimize(cp.log_sum_exp(x)),
        [
            x == np.array([1, 0]),
        ]  # NOTE you can modify this constraint to change the solution.
    )
    prob1.solve()
    print("optimal objective value", prob1.value)
    print("optimal x", x.value)

    t = cp.Variable()
    z = cp.Variable(2)
    prob2 = cp.Problem(
        cp.Minimize(t),
        [
            cp.sum(z) <= 1,
            cp.ExpCone(x - t, np.ones(2), z),
            # NOTE you can modify this constraint to change the solution.
            x == np.array([1, 0]),
        ]
    )
    prob2.solve()
    print("optimal objective value", prob2.value)
    print("optimal x", x.value)
    print("optimal t", t.value)
    print("optimal z", z.value)


log_sum_exp_impl()
