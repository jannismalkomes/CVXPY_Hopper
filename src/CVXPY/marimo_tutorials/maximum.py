# How can you represent maximum by introducing new variables and inequalities?
# maximum(x, y) equals the larger of x and y.

import cvxpy as cp


def maximum_impl():
    x = cp.Variable()
    y = cp.Variable()
    prob1 = cp.Problem(
        cp.Minimize(cp.maximum(x, y)),
        [
            x == 2,
            y == 3,
        ]  # NOTE you can modify these constraint to change the solution.
    )
    prob1.solve()
    print("optimal objective value", prob1.value)
    print("optimal x", x.value)
    print("optimal y", y.value)

    z = cp.Variable()
    prob2 = cp.Problem(
        cp.Minimize(z),
        [
            z >= x,
            z >= y,
            x == 2,
            y == 3,
        ]  # NOTE you can modify these constraint to change the solution.
    )
    prob2.solve()
    print("optimal objective value", prob2.value)
    print("optimal x", x.value)
    print("optimal y", y.value)
    print("optimal z", z.value)


maximum_impl()
