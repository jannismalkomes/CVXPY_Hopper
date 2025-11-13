# GFOLD C Code - Quick Reference

## 🚀 Interface Summary

### Parameters (5 total)
```c
r_initial[3]  // Initial position [x,y,z] in meters
v_initial[3]  // Initial velocity [vx,vy,vz] in m/s  
r_target[3]   // Target position [x,y,z] in meters
v_target[3]   // Target velocity [vx,vy,vz] in m/s
tf            // Flight time in seconds
```

### Python Quick Start
```python
import cpg_module
import numpy as np

# Setup
updated = cpg_module.cpg_updated()
params = cpg_module.cpg_params()

# Set mission
params.r_initial = np.array([0, 0, 10])  # 10m altitude
params.v_initial = np.array([0, 0, 0])   # Start from rest
params.r_target = np.array([0, 0, 0])    # Land at origin  
params.v_target = np.array([0, 0, 0])    # Zero velocity
params.tf = 8.0                          # 8 second flight

# Mark as updated
updated.r_initial = updated.v_initial = True
updated.r_target = updated.v_target = updated.tf = True

# Solve and check
result = cpg_module.solve(updated, params)
print(f"Status: {result.cpg_info.status}")  # 0 = success
print(f"Fuel: {200 - np.exp(result.cpg_info.obj_val):.1f} kg")
```

### C Quick Start  
```c
#include "cpg_solve.h"

// Set mission parameters
cpg_update_r_initial(0, 0.0);   cpg_update_r_initial(1, 0.0);   cpg_update_r_initial(2, 10.0);
cpg_update_v_initial(0, 0.0);   cpg_update_v_initial(1, 0.0);   cpg_update_v_initial(2, 0.0);
cpg_update_r_target(0, 0.0);    cpg_update_r_target(1, 0.0);    cpg_update_r_target(2, 0.0);
cpg_update_v_target(0, 0.0);    cpg_update_v_target(1, 0.0);    cpg_update_v_target(2, 0.0);
cpg_update_tf(8.0);

// Solve
cpg_solve();

// Check result
if (workspace.info.status == 0) {
    printf("Success! Fuel: %.1f kg\n", 200.0 - exp(workspace.info.obj_val));
}
```

## ⚙️ Solver Settings

### Fast & Real-Time
```c
cpg_set_solver_abstol(1e-6);
cpg_set_solver_reltol(1e-6); 
cpg_set_solver_maxit(50);
```

### High Precision
```c
cpg_set_solver_abstol(1e-12);
cpg_set_solver_reltol(1e-12);
cpg_set_solver_maxit(200);
```

## 📊 Status Codes
- `0`: Optimal solution ✅
- `1`: Infeasible problem ❌  
- `10`: Max iterations reached ⚠️

## 🎯 Performance
- **Solve Time**: ~20ms typical
- **Speedup**: 20x faster than Python
- **Memory**: Fixed allocation, embedded-friendly

## 📐 Vehicle Specs
- **Mass**: 200kg (140kg dry + 60kg fuel)
- **Thrust**: 1400-2000N (70-100% throttle)  
- **Max Tilt**: ±15° from vertical
- **Coordinate**: Z-up (z=0 is ground)