# In this exercise, try to represent nonlinear functions like abs and max
# by introducing new variables and inequalities.abs

import cvxpy as cp

# Example with abs


def abs_impl():
    x = cp.Variable()
    prob1 = cp.Problem(
        cp.Minimize(cp.abs(x)),
        [x == 2]  # NOTE you can modify this constraint to change the solution.
    )
    prob1.solve()
    print("optimal objective value", prob1.value)
    print("optimal x", x.value)

    y = cp.Variable()
    prob2 = cp.Problem(
        cp.Minimize(y),
        [
            # NOTE you can modify this constraint to change the solution.
            x == 2,
            -y <= x,
            x <= y,
        ]
    )
    prob2.solve()
    print("optimal objective value", prob2.value)
    print("optimal x", x.value)
    print("optimal y", y.value)


abs_impl()
