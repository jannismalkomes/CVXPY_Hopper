# GFOLD C Solver - Quick Start Guide

## 🚀 Run GFOLD Solver & Plot Results

### Step 0: Build the executable (if needed)
```bash
# Quick build (if already configured)
make cpg_example

# Or full rebuild from scratch
cd /path/to/code_export/c
rm -rf build/*
cd build
cmake ..
make cpg_example
```

### Step 1: Execute the C solver

From executable folder:

```bash
./cpg_example --r_initial 0 0 5 --v_initial 0 0 0 --r_target 0 0 0 --v_target 0 0 0 --tf 12 > out.txt
```

Or from high level python script path:

```bash
./c/build/cpg_example --r_initial 0 0 5 --v_initial 0 0 0 --r_target 0 0 0 --v_target 0 0 0 --tf 12 > ./c/build/out.txt
```

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
# Vertical descent (5m → 0m)
./cpg_example --r_initial 0 0 5 --r_target 0 0 0 --tf 8 > out.txt

# Horizontal hop (5m up, 10m forward)
./cpg_example --r_initial 0 0 5 --r_target 10 0 0 --tf 12 > out.txt

# High altitude descent
./cpg_example --r_initial 0 0 20 --r_target 0 0 0 --tf 15 > out.txt
```

## 📁 Files Generated
- `out.txt` - Raw solver output (all trajectory data)
- `trajectory_plots.png` - Visualization plots
- `plot_trajectory.py` - Python plotting script

**Total time**: < 1 second to solve + plot! 🎉