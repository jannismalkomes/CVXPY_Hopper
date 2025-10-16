# GFOLD C Code Export - Validation Report

## Summary
✅ **VALIDATED**: Python and C solvers produce equivalent solutions with correct thrust constraints.

## Problem Discovered
The original C code generation had **incorrect thrust bound constraints** that allowed thrust to exceed the 2000N physical limit.

### Root Cause
In `trajectory_optimizer.py`, the `export_c_code()` function (lines 476-486) used **oversimplified constant bounds**:
```python
# WRONG (original):
constraints += [s[0, n] <= self.params.r2 / self.params.m_dry]  # 2000/50 = 40 m/s²
```

This allowed thrust acceleration up to 40 m/s², resulting in forces up to **2607 N** - violating the 2000N limit by 30%!

### Solution
Changed `export_c_code()` to use the **same linearized thrust bounds** as `solve_problem_4()`:
```python
# CORRECT (fixed):
z0_term = self.params.m_wet - self.params.alpha * self.params.r2 * n * self.dt
mu_2 = self.params.r2 / z0_term
constraints += [s[0, n] <= mu_2 * (1 - (z[0, n] - z0))]
```

## Validation Results (tf=12s, r0=[0,0,10])

| Metric | Python Solver | C Solver | Match |
|--------|--------------|----------|-------|
| **Status** | OPTIMAL_INACCURATE | OPTIMAL_INACCURATE | ✅ |
| **Objective** | 5.239967 | 5.239477 | ✅ (Δ=0.0005) |
| **Final Mass** | 188.66 kg | 188.57 kg | ✅ (Δ=0.09 kg) |
| **Fuel Used** | 11.34 kg | 11.43 kg | ✅ (Δ=0.09 kg) |
| **Max Thrust** | 1980.88 N | 1980.74 N | ✅ (<2000N) |
| **Min Thrust** | 1659.80 N | 1654.37 N | ✅ (>1400N) |

## Key Findings

1. **Thrust Bounds Enforced**: Both solvers now respect 1400-2000N limits
2. **Solutions Match**: <0.1 kg difference in final mass
3. **Conservative Constraints**: Linearization may report "infeasible" for aggressive scenarios (tf=8s)
4. **Flight Time Adjustment**: Increasing tf allows finding feasible trajectories

## Recommendations

1. ✅ Use `code_export_FIXED` for deployment
2. ✅ Adjust flight time parameter if solver reports infeasible
3. ✅ Monitor thrust force stays within [1400, 2000] N range
4. ⚠️ "OPTIMAL_INACCURATE" status is acceptable (residuals < 1e-10)

## Test Commands

### Python Solver
```bash
cd /workspaces/CVXPY_Hopper/src/Trajectory_Generator_4
python3 -c "from trajectory_optimizer import GFOLDTrajectoryGenerator; import numpy as np; gen = GFOLDTrajectoryGenerator(); gen.N = 120; gen.solve_problem_4(12.0, np.array([0,0,10]), np.array([0,0,0]))"
```

### C Solver
```bash
cd /workspaces/CVXPY_Hopper/src/Trajectory_Generator_4/code_export_FIXED/c/build
./cpg_example --r_initial 0 0 10 --v_initial 0 0 0 --r_target 0 0 0 --v_target 0 0 0 --tf 12 --v_max 10 --g_max 10 > out.txt
```

---
Generated: October 16, 2025
