"""
Problem 4 Data Export Function - Usage Guide

The export_problem4_data() function provides a single, comprehensive data export
solution for when only Problem 4 (minimum fuel use) is executed.

## Function Signature:
```python
data_exporter.export_problem4_data(solution_data, filename_prefix="problem4")
```

## Parameters:
- solution_data: Dictionary containing Problem 4 solution results (from solve_problem_4())
- filename_prefix: Optional prefix for output filenames (default: "problem4")

## Generated Files:
1. {prefix}_trajectory.csv - Basic trajectory data (position, velocity, thrust, mass)
2. {prefix}_solver_stats.csv - Optimization statistics and problem metadata
3. {prefix}_detailed.csv - Comprehensive trajectory with computed quantities
4. {prefix}_interpolated.csv - High-resolution interpolated data (1000 points)
5. {prefix}_simple.csv - Simplified data (position, velocity, acceleration only)

## Usage Example:
```python
from trajectory_optimizer import GFOLDTrajectoryGenerator

# Create generator and solve Problem 4
generator = GFOLDTrajectoryGenerator()
results = generator.solve_problem_4(tf, r0, rf_target)

# Export all data with single function call
success = generator.data_exporter.export_problem4_data(
    results, filename_prefix="my_gfold_run"
)

if success:
    print("All data exported successfully!")
```

## File Contents:

### Basic Trajectory ({prefix}_trajectory.csv):
- time, pos_x, pos_y, pos_z
- vel_x, vel_y, vel_z  
- thrust_x, thrust_y, thrust_z
- log_mass, mass, thrust_magnitude

### Solver Statistics ({prefix}_solver_stats.csv):
- Problem status, solve time, iterations
- Physical parameters (masses, positions)
- Fuel consumption and efficiency metrics
- CVXPY solver details

### Detailed Trajectory ({prefix}_detailed.csv):
- All basic data plus:
- Position/velocity/thrust magnitudes
- Total acceleration components
- Thrust-to-weight ratio

### Interpolated Data ({prefix}_interpolated.csv):
- High-resolution version of detailed data
- 1000 uniformly spaced time points
- Smooth trajectories for plotting/analysis

### Simplified Data ({prefix}_simple.csv):
- Minimal dataset: time, position, velocity, acceleration
- Ideal for basic trajectory analysis
- Smaller file size

## Benefits:
- Single function call exports all relevant data
- Consistent file naming and structure
- Comprehensive coverage of trajectory and optimization data
- Works independently of Problem 3 solutions
- Robust error handling with descriptive messages
"""