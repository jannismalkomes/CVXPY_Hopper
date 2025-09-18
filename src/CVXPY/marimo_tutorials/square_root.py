# Challenge problem 1
#
# How can you represent sqrt by introducing new variables and constraints?
# Hint: use cp.SOC(t, y), which mathematically represents ||y||_2 <= t
# You may also need cp.hstack to combine multiple scalars into a vector.

import cvxpy as cp


def sqrt_impl():
    x = cp.Variable()
    prob1 = cp.Problem(
        cp.Maximize(cp.sqrt(x)),
        [
            x == 4,
        ]  # NOTE you can modify this constraint to change the solution.
    )
    prob1.solve()
    print("optimal objective value", prob1.value)
    print("optimal x", x.value)

    # Chain of equivalencies:
    # sqrt(x) >= t
    # x >= t^2
    # 4x >= (2t)^2
    # x^2 + 2x + 1 >= x^2 - 2x + 1 + (2t)^2
    # (x + 1)^2 >= (x - 1)^2 + (2t)^2
    # x + 1 >= ||(x - 1, t)||_2
    t = cp.Variable(nonneg=True)
    prob2 = cp.Problem(
        cp.Maximize(t),
        [
            cp.SOC(x + 1, cp.hstack([2*t, x - 1])),
            x == 4,
        ]  # NOTE you can modify this constraint to change the solution.
    )
    prob2.solve()
    print("optimal objective value", prob2.value)
    print("optimal x", x.value)
    print("optimal t", t.value)


sqrt_impl()
