# GFOLD Trajectory Generator - C Code Export

## Overview

This directory contains the exported C code for the GFOLD (Guidance for Fuel-Optimal Large Diverts) trajectory generator. The code was automatically generated from the CVXPY optimization problem using CVXPYgen, providing a high-performance, embedded-friendly implementation suitable for real-time trajectory generation.

### Performance Characteristics
- **Solve Time**: ~20ms (vs. ~400ms for Python CVXPY)
- **Memory Footprint**: Fixed, no dynamic allocation
- **Dependencies**: Self-contained C library with ECOS solver
- **Speedup**: ~20x faster than Python implementation

## Problem Formulation

The GFOLD algorithm solves a fuel-optimal trajectory optimization problem for spacecraft powered descent. The problem is formulated as a convex optimization with the following characteristics:

### Objective
Minimize fuel consumption (maximize final mass):
```
minimize: -log(m_final)
```

### Vehicle Parameters
- **Mass**: 200kg total (140kg dry + 60kg fuel)
- **Thrust**: 1400-2000N (70-100% throttle)
- **Specific Impulse**: 203.94s
- **Max Acceleration**: 10g structural limit
- **Max Velocity**: 10 m/s operational limit

### Constraints
1. **Dynamics**: 6-DOF point mass with gravity and Coriolis effects
2. **Thrust Bounds**: 1400N ≤ |T| ≤ 2000N
3. **Thrust Pointing**: ±15° maximum tilt angle from vertical
4. **State Bounds**: Position, velocity, and acceleration limits
5. **Boundary Conditions**: Initial and final position/velocity

## Interface Documentation

### Runtime Parameters

The C solver accepts 5 runtime parameters that can be changed for each solve:

| Parameter | Type | Size | Description | Units |
|-----------|------|------|-------------|-------|
| `r_initial` | `double[3]` | 3 | Initial position [x, y, z] | meters |
| `v_initial` | `double[3]` | 3 | Initial velocity [vx, vy, vz] | m/s |
| `r_target` | `double[3]` | 3 | Target position [x, y, z] | meters |
| `v_target` | `double[3]` | 3 | Target velocity [vx, vy, vz] | m/s |
| `tf` | `double` | 1 | Flight time | seconds |

**Coordinate System**: Z-up (z=0 is ground level, positive z is altitude)

### Python Interface

#### Basic Usage
```python
import cpg_module
import numpy as np

# Create parameter objects
updated = cpg_module.cpg_updated()
params = cpg_module.cpg_params()

# Set parameters
params.r_initial = np.array([0.0, 0.0, 10.0])  # Start 10m high
params.v_initial = np.array([0.0, 0.0, 0.0])   # From rest
params.r_target = np.array([0.0, 0.0, 0.0])    # Land at origin
params.v_target = np.array([0.0, 0.0, 0.0])    # Zero velocity landing
params.tf = 8.0                                 # 8 second flight

# Mark parameters as updated
updated.r_initial = True
updated.v_initial = True
updated.r_target = True
updated.v_target = True
updated.tf = True

# Solve
result = cpg_module.solve(updated, params)

# Check results
if result.cpg_info.status == 0:  # 0 = optimal
    print(f"Solve time: {result.cpg_info.time*1000:.2f} ms")
    print(f"Fuel consumed: {200 - np.exp(result.cpg_info.obj_val):.2f} kg")
```

#### Result Structure
```python
result.cpg_info.status      # Solver status (0=optimal, 1=infeasible, etc.)
result.cpg_info.iter        # Number of iterations
result.cpg_info.time        # Solve time in seconds
result.cpg_info.obj_val     # Objective value (log final mass)
result.cpg_info.pri_res     # Primal residual
result.cpg_info.dua_res     # Dual residual

result.cpg_prim.state       # State trajectory (flattened, 480 elements)
result.cpg_prim.control     # Control trajectory (flattened, 240 elements)
result.cpg_prim.log_mass    # Log mass trajectory (80 elements)
result.cpg_prim.thrust_slack # Thrust slack variables
```

### C Interface

#### Function Declarations
```c
#include "cpg_solve.h"

// Update parameters
void cpg_update_r_initial(cpg_int idx, cpg_float val);
void cpg_update_v_initial(cpg_int idx, cpg_float val);
void cpg_update_r_target(cpg_int idx, cpg_float val);
void cpg_update_v_target(cpg_int idx, cpg_float val);
void cpg_update_tf(cpg_float val);

// Solve
void cpg_solve();

// Access results
extern cpg_workspace workspace;  // Contains all solution data
```

#### Basic C Usage
```c
#include "cpg_solve.h"

int main() {
    // Set parameters
    cpg_update_r_initial(0, 0.0);   // x_initial
    cpg_update_r_initial(1, 0.0);   // y_initial  
    cpg_update_r_initial(2, 10.0);  // z_initial
    
    cpg_update_v_initial(0, 0.0);   // vx_initial
    cpg_update_v_initial(1, 0.0);   // vy_initial
    cpg_update_v_initial(2, 0.0);   // vz_initial
    
    cpg_update_r_target(0, 0.0);    // x_target
    cpg_update_r_target(1, 0.0);    // y_target
    cpg_update_r_target(2, 0.0);    // z_target
    
    cpg_update_v_target(0, 0.0);    // vx_target
    cpg_update_v_target(1, 0.0);    // vy_target
    cpg_update_v_target(2, 0.0);    // vz_target
    
    cpg_update_tf(8.0);             // flight_time
    
    // Solve
    cpg_solve();
    
    // Check results
    if (workspace.info.status == 0) {
        printf("Optimal solution found!\n");
        printf("Solve time: %.3f ms\n", workspace.info.runtime * 1000);
        printf("Final mass: %.2f kg\n", exp(workspace.info.obj_val));
    }
    
    return 0;
}
```

## Solver Settings

### Available Settings

| Setting | Default | Description | Valid Range |
|---------|---------|-------------|-------------|
| `feastol` | 1e-8 | Feasibility tolerance | > 0 |
| `abstol` | 1e-8 | Absolute tolerance | > 0 |
| `reltol` | 1e-8 | Relative tolerance | > 0 |
| `feastol_inacc` | 1e-4 | Inaccurate feasibility tolerance | > 0 |
| `abstol_inacc` | 1e-4 | Inaccurate absolute tolerance | > 0 |
| `reltol_inacc` | 1e-4 | Inaccurate relative tolerance | > 0 |
| `maxit` | 100 | Maximum iterations | > 0 |

### Python Settings Interface
```python
# Set solver tolerances for higher precision
cpg_module.set_solver_abstol(1e-10)
cpg_module.set_solver_reltol(1e-10)
cpg_module.set_solver_feastol(1e-10)

# Set maximum iterations
cpg_module.set_solver_maxit(200)

# Reset to defaults
cpg_module.set_solver_default_settings()
```

### C Settings Interface
```c
// Adjust tolerances for real-time constraints
cpg_set_solver_abstol(1e-6);     // Faster, less precise
cpg_set_solver_reltol(1e-6);
cpg_set_solver_feastol(1e-6);
cpg_set_solver_maxit(50);        // Limit iterations

// For high precision
cpg_set_solver_abstol(1e-12);    // Slower, more precise
cpg_set_solver_reltol(1e-12);
cpg_set_solver_feastol(1e-12);
```

## Flight Software Integration

### Real-Time Considerations

1. **Deterministic Timing**: Solver typically converges in 10-15 iterations (~20ms)
2. **Memory**: Fixed allocation, no dynamic memory management
3. **Thread Safety**: Not thread-safe, requires external synchronization
4. **Error Handling**: Always check `status` before using results

### Integration Pattern
```c
// Flight software integration example
typedef struct {
    double r_current[3];
    double v_current[3];
    double r_landing[3];
    double v_landing[3];
    double time_to_landing;
} GuidanceCommand;

int generate_trajectory(GuidanceCommand* cmd, double* thrust_profile) {
    // Update solver with current state
    cpg_update_r_initial(0, cmd->r_current[0]);
    cpg_update_r_initial(1, cmd->r_current[1]);
    cpg_update_r_initial(2, cmd->r_current[2]);
    
    cpg_update_v_initial(0, cmd->v_current[0]);
    cpg_update_v_initial(1, cmd->v_current[1]);
    cpg_update_v_initial(2, cmd->v_current[2]);
    
    cpg_update_r_target(0, cmd->r_landing[0]);
    cpg_update_r_target(1, cmd->r_landing[1]);
    cpg_update_r_target(2, cmd->r_landing[2]);
    
    cpg_update_v_target(0, cmd->v_landing[0]);
    cpg_update_v_target(1, cmd->v_landing[1]);
    cpg_update_v_target(2, cmd->v_landing[2]);
    
    cpg_update_tf(cmd->time_to_landing);
    
    // Solve
    cpg_solve();
    
    // Check for success
    if (workspace.info.status != 0) {
        return -1;  // Failed to find solution
    }
    
    // Extract thrust commands (first few time steps)
    // Control is [ux, uy, uz] for each time step
    for (int i = 0; i < 10; i++) {  // Next 10 time steps
        thrust_profile[3*i + 0] = workspace.primal.control[3*i + 0];
        thrust_profile[3*i + 1] = workspace.primal.control[3*i + 1]; 
        thrust_profile[3*i + 2] = workspace.primal.control[3*i + 2];
    }
    
    return 0;  // Success
}
```

### Error Codes
- `0`: Optimal solution found
- `1`: Infeasible problem
- `2`: Unbounded problem  
- `10`: Maximum iterations reached
- `-1`: Other solver error

## MATLAB Integration

### MEX Interface
Create a MEX wrapper for MATLAB integration:

```matlab
% gfold_solve.c - MEX wrapper
#include "mex.h"
#include "cpg_solve.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // Input validation
    if (nrhs != 5) {
        mexErrMsgTxt("Usage: [status, time, fuel] = gfold_solve(r_init, v_init, r_target, v_target, tf)");
    }
    
    // Extract parameters
    double *r_init = mxGetPr(prhs[0]);
    double *v_init = mxGetPr(prhs[1]);
    double *r_target = mxGetPr(prhs[2]);
    double *v_target = mxGetPr(prhs[3]);
    double tf = mxGetScalar(prhs[4]);
    
    // Update solver
    for (int i = 0; i < 3; i++) {
        cpg_update_r_initial(i, r_init[i]);
        cpg_update_v_initial(i, v_init[i]);
        cpg_update_r_target(i, r_target[i]);
        cpg_update_v_target(i, v_target[i]);
    }
    cpg_update_tf(tf);
    
    // Solve
    cpg_solve();
    
    // Return results
    plhs[0] = mxCreateDoubleScalar(workspace.info.status);
    plhs[1] = mxCreateDoubleScalar(workspace.info.runtime);
    plhs[2] = mxCreateDoubleScalar(200.0 - exp(workspace.info.obj_val));
}
```

### MATLAB Usage
```matlab
% Compile MEX file
mex gfold_solve.c cpg_solve.c cpg_workspace.c -I./include -L./lib -lecos

% Use in MATLAB
r_initial = [0; 0; 10];     % Start position
v_initial = [0; 0; 0];      % Start velocity  
r_target = [0; 0; 0];       % Landing position
v_target = [0; 0; 0];       % Landing velocity
tf = 8.0;                   % Flight time

[status, solve_time, fuel_used] = gfold_solve(r_initial, v_initial, r_target, v_target, tf);

if status == 0
    fprintf('Optimal trajectory found!\n');
    fprintf('Solve time: %.2f ms\n', solve_time * 1000);
    fprintf('Fuel consumed: %.2f kg\n', fuel_used);
else
    fprintf('Failed to find solution (status: %d)\n', status);
end
```

### Simulink Integration
```matlab
% Create S-function for Simulink
function gfold_sfunc(block)
    setup(block);
end

function setup(block)
    % Register ports
    block.NumInputPorts = 5;    % r_init, v_init, r_target, v_target, tf
    block.NumOutputPorts = 3;   % status, solve_time, fuel_used
    
    % Set port properties
    for i = 1:3
        block.InputPort(i).Dimensions = 3;  % 3D vectors
    end
    block.InputPort(4).Dimensions = 3;      % v_target
    block.InputPort(5).Dimensions = 1;      % tf scalar
    
    block.OutputPort(1).Dimensions = 1;     % status
    block.OutputPort(2).Dimensions = 1;     % solve_time  
    block.OutputPort(3).Dimensions = 1;     % fuel_used
    
    block.RegBlockMethod('Outputs', @Outputs);
end

function Outputs(block)
    % Call MEX function
    [status, time, fuel] = gfold_solve(...
        block.InputPort(1).Data, ...   % r_initial
        block.InputPort(2).Data, ...   % v_initial
        block.InputPort(3).Data, ...   % r_target
        block.InputPort(4).Data, ...   % v_target
        block.InputPort(5).Data);      % tf
    
    block.OutputPort(1).Data = status;
    block.OutputPort(2).Data = time;
    block.OutputPort(3).Data = fuel;
end
```

## Build Instructions

### Standalone C Library
```bash
cd c/
mkdir build && cd build
cmake ..
make
```

### Python Extension (Already Built)
```bash
python setup.py build_ext --inplace
```

### Cross-Compilation Example
```bash
# For ARM embedded systems
cd c/build
cmake -DCMAKE_TOOLCHAIN_FILE=arm-toolchain.cmake ..
make
```

## Troubleshooting

### Common Issues

1. **Infeasible Problem (status=1)**
   - Check if trajectory is physically possible
   - Verify thrust and time constraints
   - Ensure landing site is reachable

2. **Slow Convergence**
   - Increase solver tolerances for faster solution
   - Check parameter scaling (distances in meters, not km)
   - Verify reasonable flight time

3. **Build Errors**
   - Ensure CMake 3.10+ is installed
   - Check compiler supports C99 standard
   - Verify all include paths are correct

### Performance Tuning

1. **For Real-Time Systems**: Use looser tolerances (1e-6)
2. **For High Precision**: Use tighter tolerances (1e-12)
3. **For Embedded**: Limit maximum iterations (20-50)

## License

This generated code is subject to the same license as the original CVXPY and CVXPYgen libraries. The ECOS solver is distributed under GPLv3 license.

---

Generated by CVXPYgen on October 9, 2025
Original Python implementation: GFOLD Trajectory Generator