# How can you represent minimum by introducing new variables and inequalities?
# minimum(x, y) equals the smaller of x and y.

import cvxpy as cp


def minimum_impl():
    x = cp.Variable()
    y = cp.Variable()
    prob1 = cp.Problem(
        cp.Maximize(cp.minimum(x, y)),
        [
            x == -1,
            y == 1,
        ]  # NOTE you can modify these constraint to change the solution.
    )
    prob1.solve()
    print("optimal objective value", prob1.value)
    print("optimal x", x.value)
    print("optimal y", y.value)

    z = cp.Variable()
    prob2 = cp.Problem(
        cp.Maximize(z),
        [
            z <= x,
            z <= y,
            x == -1,
            y == 1,
        ]  # NOTE you can modify these constraint to change the solution.
    )
    prob2.solve()
    print("optimal objective value", prob2.value)
    print("optimal x", x.value)
    print("optimal y", y.value)
    print("optimal z", z.value)


minimum_impl()
