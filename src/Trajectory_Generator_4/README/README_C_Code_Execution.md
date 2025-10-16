# GFOLD C Solver - Quick Start Guide

## 🚀 Run GFOLD Solver & Plot Results

### Step 0: Build the executable (if needed)

**Directory:** Navigate to `code_export/c/build/` (or wherever your generated C code is located)

```bash
# Quick build (if already configured) - from code_export/c/build/ directory
make cpg_example

# Or full rebuild from scratch
cd /path/to/code_export/c
rm -rf build/*
cd build
cmake ..
make cpg_example
```

**Example paths:**
```bash
# If using code_export_with_limits
cd /workspaces/CVXPY_Hopper/src/Trajectory_Generator_4/code_export_with_limits/c/build

# If using code_export_1_modified
cd /workspaces/CVXPY_Hopper/src/Trajectory_Generator_4/code_export_1_modified/c/build
```

### Step 1: Execute the C solver

```bash
# From executable folder
cd /workspaces/CVXPY_Hopper/src/Trajectory_Generator_4/code_export_FIXED/c/build && ./cpg_example --r_initial 0 0 10 --v_initial 0 0 0 --r_target 0 0 0 --v_target 0 0 0 --tf 12 --v_max 10 --g_max 10

# Or from project root
cd /workspaces/CVXPY_Hopper/src/Trajectory_Generator_4/code_export_FIXED && ./c/build/cpg_example --r_initial 0 0 10 --v_initial 0 0 0 --r_target 0 0 0 --v_target 0 0 0 --tf 12 --v_max 10 --g_max 10
```

**Parameters:**
- `--r_initial X Y Z` - Initial position [m]
- `--v_initial X Y Z` - Initial velocity [m/s]
- `--r_target X Y Z` - Target position [m]
- `--v_target X Y Z` - Target velocity [m/s]
- `--tf TIME` - Flight time [s]
- `--v_max VEL` - Maximum velocity [m/s] (default: 10)
- `--g_max ACC` - Maximum acceleration [g] (default: 10)
- `--feastol TOL` - Feasibility tolerance (default: 1e-8)
- `--abstol TOL` - Absolute tolerance (default: 1e-8)
- `--reltol TOL` - Relative tolerance (default: 1e-8)

### Step 2: Plot the trajectory
```bash
python plot_trajectory.py
```

## 📊 What You Get
- **Mass trajectory**: Fuel consumption over time
- **Thrust profile**: Engine commands vs time  
- **Altitude profile**: Height above ground vs time
- **Output files**: `trajectory_plots.png` and `out.txt`

## 🎯 Example Commands

```bash
# Vertical descent (5m → 0m) with default limits
./cpg_example --r_initial 0 0 5 --r_target 0 0 0 --tf 8 > out.txt

# Vertical descent with custom velocity limit (5 m/s)
./cpg_example --r_initial 0 0 5 --r_target 0 0 0 --tf 8 --v_max 5 > out.txt

# Vertical descent with custom acceleration limit (5 g)
./cpg_example --r_initial 0 0 5 --r_target 0 0 0 --tf 8 --g_max 5 > out.txt

# Horizontal hop (5m up, 10m forward) with tight constraints
./cpg_example --r_initial 0 0 5 --r_target 10 0 0 --tf 12 --v_max 8 --g_max 6 > out.txt

# High altitude descent with relaxed limits
./cpg_example --r_initial 0 0 20 --r_target 0 0 0 --tf 15 --v_max 15 --g_max 15 > out.txt

# Fast solve with relaxed tolerances (2-3x speedup, minimal accuracy loss)
./cpg_example --r_initial 0 0 10 --r_target 0 0 0 --tf 12 --feastol 1e-6 --abstol 1e-6 --reltol 1e-6 > out.txt

# Very fast solve with aggressive tolerances (5x speedup, for real-time control)
./cpg_example --r_initial 0 0 10 --r_target 0 0 0 --tf 12 --feastol 1e-4 --abstol 1e-4 --reltol 1e-4 > out.txt
```

## 📁 Files Generated
- `out.txt` - Raw solver output (all trajectory data)
- `trajectory_plots.png` - Visualization plots
- `plot_trajectory.py` - Python plotting script

**Total time**: < 1 second to solve + plot! 🎉