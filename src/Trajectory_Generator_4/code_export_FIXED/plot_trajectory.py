#!/usr/bin/env python3
"""
GFOLD Trajectory Plotter

This script parses the GFOLD C solver output and creates plots for:
- Mass trajectory over time
- Thrust magnitude over time  
- Height (altitude) over time
- Velocity trajectories

Usage: python plot_trajectory.py [path/to/out.txt]
"""

import numpy as np
import matplotlib.pyplot as plt
import re
import sys
import os


def parse_gfold_output(filename):
    """
    Parse the GFOLD C solver output file.

    Returns:
        dict: Contains parsed trajectory data
    """
    with open(filename, 'r') as f:
        content = f.read()

    # Parse parameters from header
    params = {}

    # Parse target parameters from the header
    target_pos_match = re.search(
        r'Target position:\s+\[([\d.-]+),\s*([\d.-]+),\s*([\d.-]+)\]\s+m', content)
    if target_pos_match:
        params['target_position'] = [float(target_pos_match.group(1)),
                                     float(target_pos_match.group(2)),
                                     float(target_pos_match.group(3))]
    else:
        params['target_position'] = [0.0, 0.0, 0.0]  # Default

    target_vel_match = re.search(
        r'Target velocity:\s+\[([\d.-]+),\s*([\d.-]+),\s*([\d.-]+)\]\s+m/s', content)
    if target_vel_match:
        params['target_velocity'] = [float(target_vel_match.group(1)),
                                     float(target_vel_match.group(2)),
                                     float(target_vel_match.group(3))]
    else:
        params['target_velocity'] = [0.0, 0.0, 0.0]  # Default

    param_match = re.search(r'Flight time:\s+([\d.]+)\s+s', content)
    if param_match:
        params['flight_time'] = float(param_match.group(1))
    else:
        params['flight_time'] = 12.0  # Default

    # Parse objective value
    obj_match = re.search(r'obj = ([\d.-]+)', content)
    if obj_match:
        params['objective'] = float(obj_match.group(1))

    # Parse solver status information (machine-readable format)
    feasible_match = re.search(r'feasible = (true|false)', content)
    if feasible_match:
        params['is_feasible'] = feasible_match.group(1) == 'true'
    else:
        params['is_feasible'] = None

    status_code_match = re.search(r'status_code = ([+-]?\d+)', content)
    if status_code_match:
        params['status_code'] = int(status_code_match.group(1))
    else:
        params['status_code'] = None

    status_msg_match = re.search(r'status_message = (\w+)', content)
    if status_msg_match:
        params['status_message'] = status_msg_match.group(1)
    else:
        params['status_message'] = None

    iterations_match = re.search(r'iterations = (\d+)', content)
    if iterations_match:
        params['iterations'] = int(iterations_match.group(1))
    else:
        params['iterations'] = None

    pri_res_match = re.search(r'primal_residual = ([\d.e+-]+)', content)
    if pri_res_match:
        params['primal_residual'] = float(pri_res_match.group(1))
    else:
        params['primal_residual'] = None

    dual_res_match = re.search(r'dual_residual = ([\d.e+-]+)', content)
    if dual_res_match:
        params['dual_residual'] = float(dual_res_match.group(1))
    else:
        params['dual_residual'] = None

    # Parse log_mass values
    log_mass_pattern = r'log_mass\[(\d+)\] = ([\d.-]+)'
    log_mass_matches = re.findall(log_mass_pattern, content)
    log_mass = np.array([float(match[1]) for match in log_mass_matches])

    # Parse state values (position and velocity)
    state_pattern = r'state\[(\d+)\] = ([\d.-]+)'
    state_matches = re.findall(state_pattern, content)
    state_values = np.array([float(match[1]) for match in state_matches])

    # Parse control values (thrust)
    control_pattern = r'control\[(\d+)\] = ([\d.-]+)'
    control_matches = re.findall(control_pattern, content)
    control_values = np.array([float(match[1]) for match in control_matches])

    # Determine number of time steps
    N = len(log_mass)

    # Create time vector
    time = np.linspace(0, params['flight_time'], N)

    # Convert log mass to actual mass
    mass = np.exp(log_mass)

    # Reshape state array (6 states per timestep: x, y, z, vx, vy, vz)
    if len(state_values) >= 6 * N:
        state_array = state_values[:6*N].reshape(N, 6)
        position = state_array[:, :3]  # x, y, z positions
        velocity = state_array[:, 3:]  # vx, vy, vz velocities
        height = position[:, 2]        # z position (altitude)
    else:
        print(f"Warning: Expected {6*N} state values, got {len(state_values)}")
        position = np.zeros((N, 3))
        velocity = np.zeros((N, 3))
        height = np.zeros(N)

    # Reshape control array (3 controls per timestep: Fx, Fy, Fz)
    # Note: control values are thrust ACCELERATION [m/s²], not force [N]
    if len(control_values) >= 3 * N:
        control_array = control_values[:3*N].reshape(N, 3)
        thrust_accel_vector = control_array  # Fx, Fy, Fz accelerations [m/s²]
        thrust_accel_magnitude = np.linalg.norm(thrust_accel_vector, axis=1)
        # Vertical thrust acceleration component
        thrust_accel_z = control_array[:, 2]

        # Convert thrust acceleration to thrust force: F = m * a
        thrust_vector = thrust_accel_vector * mass[:, np.newaxis]  # Force [N]
        thrust_magnitude = np.linalg.norm(
            thrust_vector, axis=1)  # Total force magnitude [N]
        thrust_z = thrust_vector[:, 2]  # Vertical thrust component [N]
    else:
        print(
            f"Warning: Expected {3*N} control values, got {len(control_values)}")
        thrust_magnitude = np.zeros(N)
        thrust_z = np.zeros(N)

    return {
        'time': time,
        'mass': mass,
        'height': height,
        'position': position,
        'velocity': velocity,
        'thrust_magnitude': thrust_magnitude,
        'thrust_z': thrust_z,
        'params': params,
        'N': N
    }


def check_target_goals(data, tolerance_pos=0.1, tolerance_vel=0.1):
    """
    Check if the final position and velocity meet the target goals.

    Args:
        data: Parsed trajectory data
        tolerance_pos: Position tolerance in meters
        tolerance_vel: Velocity tolerance in m/s

    Returns:
        dict: Results of goal checking
    """
    # Final achieved values
    final_position = data['position'][-1]
    final_velocity = data['velocity'][-1]

    # Target values
    target_position = np.array(data['params']['target_position'])
    target_velocity = np.array(data['params']['target_velocity'])

    # Calculate errors
    pos_error = np.linalg.norm(final_position - target_position)
    vel_error = np.linalg.norm(final_velocity - target_velocity)

    # Check if goals are met
    pos_achieved = pos_error <= tolerance_pos
    vel_achieved = vel_error <= tolerance_vel

    return {
        'position_achieved': pos_achieved,
        'velocity_achieved': vel_achieved,
        'position_error': pos_error,
        'velocity_error': vel_error,
        'final_position': final_position,
        'final_velocity': final_velocity,
        'target_position': target_position,
        'target_velocity': target_velocity,
        'tolerance_pos': tolerance_pos,
        'tolerance_vel': tolerance_vel
    }


def plot_trajectories(data):
    """
    Create plots for mass, thrust, and height trajectories.
    """
    # Set dark mode style
    plt.style.use('dark_background')

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('GFOLD Trajectory Analysis', fontsize=16,
                 fontweight='bold', color='white')

    # Set dark background for the figure
    fig.patch.set_facecolor('#1e1e1e')

    time = data['time']

    # Plot 1: Mass trajectory
    ax1 = axes[0, 0]
    ax1.set_facecolor('#2d2d2d')
    ax1.plot(time, data['mass'], color='#60a5fa',
             linewidth=2, label='Spacecraft Mass')
    ax1.set_xlabel('Time [s]', color='white')
    ax1.set_ylabel('Mass [kg]', color='white')
    ax1.set_title('Mass Trajectory (Fuel Consumption)', color='white')
    ax1.grid(True, alpha=0.2, color='gray')
    ax1.legend(facecolor='#2d2d2d', edgecolor='gray')
    ax1.tick_params(colors='white')
    ax1.spines['bottom'].set_color('gray')
    ax1.spines['top'].set_color('gray')
    ax1.spines['left'].set_color('gray')
    ax1.spines['right'].set_color('gray')

    # Add fuel consumption annotation
    initial_mass = data['mass'][0]
    final_mass = data['mass'][-1]
    fuel_used = initial_mass - final_mass
    ax1.text(0.7, 0.95, f'Fuel used: {fuel_used:.2f} kg\n({fuel_used/initial_mass*100:.1f}% of initial)',
             transform=ax1.transAxes, bbox=dict(
                 boxstyle="round,pad=0.3", facecolor="#3b82f6", edgecolor='white', linewidth=1),
             verticalalignment='top', color='white')

    # Plot 2: Height trajectory
    ax2 = axes[0, 1]
    ax2.set_facecolor('#2d2d2d')
    ax2.plot(time, data['height'], color='#34d399',
             linewidth=2, label='Altitude')
    ax2.set_xlabel('Time [s]', color='white')
    ax2.set_ylabel('Height [m]', color='white')
    ax2.set_title('Altitude Profile', color='white')
    ax2.grid(True, alpha=0.2, color='gray')
    ax2.legend(facecolor='#2d2d2d', edgecolor='gray')
    ax2.axhline(y=0, color='#ef4444', linestyle='--',
                alpha=0.7, label='Ground level')
    ax2.tick_params(colors='white')
    ax2.spines['bottom'].set_color('gray')
    ax2.spines['top'].set_color('gray')
    ax2.spines['left'].set_color('gray')
    ax2.spines['right'].set_color('gray')

    # Plot 3: Thrust magnitude
    ax3 = axes[1, 0]
    ax3.set_facecolor('#2d2d2d')
    ax3.plot(time, data['thrust_magnitude'], color='#f87171',
             linewidth=2, label='Total Thrust')
    ax3.plot(time, data['thrust_z'], color='#fb923c', linewidth=2,
             linestyle='--', label='Vertical Thrust')
    ax3.set_xlabel('Time [s]', color='white')
    ax3.set_ylabel('Thrust [N]', color='white')
    ax3.set_title('Thrust Profile', color='white')
    ax3.grid(True, alpha=0.2, color='gray')
    ax3.legend(facecolor='#2d2d2d', edgecolor='gray')
    ax3.tick_params(colors='white')
    ax3.spines['bottom'].set_color('gray')
    ax3.spines['top'].set_color('gray')
    ax3.spines['left'].set_color('gray')
    ax3.spines['right'].set_color('gray')

    # Add thrust statistics
    avg_thrust = np.mean(data['thrust_magnitude'])
    max_thrust = np.max(data['thrust_magnitude'])
    ax3.text(0.7, 0.95, f'Avg thrust: {avg_thrust:.1f} N\nMax thrust: {max_thrust:.1f} N',
             transform=ax3.transAxes, bbox=dict(
                 boxstyle="round,pad=0.3", facecolor="#f59e0b", edgecolor='white', linewidth=1),
             verticalalignment='top', color='white')

    # Plot 4: Velocity trajectory (using actual solver velocities, NO post-processing)
    ax4 = axes[1, 1]
    ax4.set_facecolor('#2d2d2d')
    ax4.plot(time, data['velocity'][:, 0],
             color='#f87171', linewidth=2, label='Vel X')
    ax4.plot(time, data['velocity'][:, 1],
             color='#34d399', linewidth=2, label='Vel Y')
    ax4.plot(time, data['velocity'][:, 2],
             color='#60a5fa', linewidth=2, label='Vel Z')
    ax4.set_xlabel('Time [s]', color='white')
    ax4.set_ylabel('Velocity [m/s]', color='white')
    ax4.set_title('Velocity Components (Solver Output)', color='white')
    ax4.grid(True, alpha=0.2, color='gray')
    ax4.legend(facecolor='#2d2d2d', edgecolor='gray')
    ax4.tick_params(colors='white')
    ax4.spines['bottom'].set_color('gray')
    ax4.spines['top'].set_color('gray')
    ax4.spines['left'].set_color('gray')
    ax4.spines['right'].set_color('gray')

    # Add velocity constraint warnings
    max_vel = np.max(np.linalg.norm(data['velocity'], axis=1))
    if max_vel > 20.0:  # Reasonable velocity limit for landing
        ax4.text(0.05, 0.95, f'⚠️ HIGH VELOCITY\nMax: {max_vel:.1f} m/s',
                 transform=ax4.transAxes, bbox=dict(
                     boxstyle="round,pad=0.3", facecolor="#dc2626", edgecolor='white', linewidth=1.5),
                 verticalalignment='top', color='white', fontweight='bold')
    elif max_vel > 5.0:
        ax4.text(0.05, 0.95, f'⚡ Velocity OK\nMax: {max_vel:.1f} m/s',
                 transform=ax4.transAxes, bbox=dict(
                     boxstyle="round,pad=0.3", facecolor="#f59e0b", edgecolor='white', linewidth=1.5),
                 verticalalignment='top', color='white')

    plt.tight_layout()
    return fig


def print_summary(data):
    """
    Print trajectory summary statistics.
    """
    print("\n" + "="*50)
    print("🚀 GFOLD TRAJECTORY SUMMARY")
    print("="*50)

    # Solver status and feasibility (most important!)
    print(f"\n🔍 SOLVER STATUS:")
    if data['params']['is_feasible'] is not None:
        if data['params']['is_feasible']:
            print(f"Solution:        ✅ FEASIBLE")
        else:
            print(f"Solution:        ❌ INFEASIBLE")
            print(f"⚠️  WARNING: Results below are meaningless!")
            print(f"   Problem has impossible constraints:")
            print(f"   - Flight time too short for given distance")
            print(f"   - Target velocity/position unreachable")
            print(f"   - Thrust limits too restrictive")
    else:
        print(f"Solution:        ❓ Unknown feasibility")

    if data['params']['status_code'] is not None:
        print(f"Status code:     {data['params']['status_code']}")
        if data['params']['status_message']:
            print(f"Status:          {data['params']['status_message']}")
        if data['params']['iterations'] is not None:
            print(f"Iterations:      {data['params']['iterations']}")
        if data['params']['primal_residual'] is not None:
            print(f"Primal residual: {data['params']['primal_residual']:.2e}")
        if data['params']['dual_residual'] is not None:
            print(f"Dual residual:   {data['params']['dual_residual']:.2e}")
    else:
        print(f"Status:          ❓ No solver status found")

    # Check target goals
    goal_check = check_target_goals(data)

    # Mission parameters
    print(f"\nFlight time:     {data['params']['flight_time']:.1f} s")
    print(f"Time steps:      {data['N']}")
    print(f"Time resolution: {data['time'][1] - data['time'][0]:.3f} s")

    # Target goal verification
    print(f"\n🎯 TARGET GOAL VERIFICATION:")
    print(
        f"Target position: [{goal_check['target_position'][0]:.2f}, {goal_check['target_position'][1]:.2f}, {goal_check['target_position'][2]:.2f}] m")
    print(
        f"Final position:  [{goal_check['final_position'][0]:.2f}, {goal_check['final_position'][1]:.2f}, {goal_check['final_position'][2]:.2f}] m")
    print(
        f"Position error:  {goal_check['position_error']:.4f} m (tolerance: {goal_check['tolerance_pos']:.2f} m)")
    print(
        f"Position goal:   {'✅ ACHIEVED' if goal_check['position_achieved'] else '❌ FAILED'}")

    print(
        f"\nTarget velocity: [{goal_check['target_velocity'][0]:.2f}, {goal_check['target_velocity'][1]:.2f}, {goal_check['target_velocity'][2]:.2f}] m/s")
    print(
        f"Final velocity:  [{goal_check['final_velocity'][0]:.2f}, {goal_check['final_velocity'][1]:.2f}, {goal_check['final_velocity'][2]:.2f}] m/s")
    print(
        f"Velocity error:  {goal_check['velocity_error']:.4f} m/s (tolerance: {goal_check['tolerance_vel']:.2f} m/s)")
    print(
        f"Velocity goal:   {'✅ ACHIEVED' if goal_check['velocity_achieved'] else '❌ FAILED'}")

    # Overall mission success
    mission_success = goal_check['position_achieved'] and goal_check['velocity_achieved']
    print(
        f"\n🏆 MISSION STATUS: {'✅ SUCCESS' if mission_success else '❌ FAILED'}")

    # Mass analysis
    initial_mass = data['mass'][0]
    final_mass = data['mass'][-1]
    fuel_used = initial_mass - final_mass
    print(f"\n📊 MASS ANALYSIS:")
    print(f"Initial mass:    {initial_mass:.2f} kg")
    print(f"Final mass:      {final_mass:.2f} kg")
    print(
        f"Fuel consumed:   {fuel_used:.2f} kg ({fuel_used/initial_mass*100:.1f}%)")

    # Trajectory analysis
    print(f"\n📈 TRAJECTORY ANALYSIS:")
    print(f"Initial height:  {data['height'][0]:.2f} m")
    print(f"Final height:    {data['height'][-1]:.2f} m")
    print(f"Max height:      {np.max(data['height']):.2f} m")

    # Velocity constraint analysis (using actual solver velocities)
    velocity_magnitudes = np.linalg.norm(data['velocity'], axis=1)
    max_velocity = np.max(velocity_magnitudes)
    final_velocity_mag = np.linalg.norm(data['velocity'][-1])

    print(f"\n🚀 VELOCITY ANALYSIS:")
    print(f"Max velocity:    {max_velocity:.2f} m/s")
    print(f"Final velocity:  {final_velocity_mag:.4f} m/s")

    # Check for velocity violations
    if max_velocity > 20.0:
        print(f"⚠️  VELOCITY VIOLATION: Exceeds safe limits (>20 m/s)")
    elif max_velocity > 10.0:
        print(f"⚡ High velocity detected (>10 m/s) - verify trajectory")
    else:
        print(f"✅ Velocity within reasonable limits")

    # Thrust analysis
    print(f"\n⚡ THRUST ANALYSIS:")
    print(f"Average thrust:  {np.mean(data['thrust_magnitude']):.2f} N")
    print(f"Maximum thrust:  {np.max(data['thrust_magnitude']):.2f} N")
    print(f"Minimum thrust:  {np.min(data['thrust_magnitude']):.2f} N")

    # Performance metrics
    thrust_to_weight = np.mean(
        data['thrust_magnitude']) / (np.mean(data['mass']) * 9.81)
    print(f"\n🚀 PERFORMANCE:")
    print(f"Avg T/W ratio:   {thrust_to_weight:.2f}")
    print(f"Objective value: {data['params'].get('objective', 'N/A')}")

    return goal_check


def main():
    """
    Main function to parse arguments and create plots.
    """
    # Parse command line arguments
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        # Default path relative to where the script is located
        script_dir = os.path.dirname(os.path.abspath(__file__))
        filename = os.path.join(script_dir, 'c', 'build', 'out.txt')

    # Check if file exists
    if not os.path.exists(filename):
        print(f"Error: File '{filename}' not found!")
        print(f"Usage: python {sys.argv[0]} [output_file.txt]")
        print(f"Or run from code_export directory after generating out.txt")
        sys.exit(1)

    print(f"📖 Parsing GFOLD output from: {filename}")

    try:
        # Parse the output file
        data = parse_gfold_output(filename)

        # Print summary and get goal check results
        goal_check = print_summary(data)

        # Create plots
        print(f"\n📊 Creating trajectory plots...")
        fig = plot_trajectories(data)

        # Save plots
        output_name = os.path.splitext(filename)[0] + '_plots.png'
        fig.savefig(output_name, dpi=300, bbox_inches='tight')
        print(f"✅ Plots saved to: {output_name}")

        # Show plots
        plt.show()

    except Exception as e:
        print(f"❌ Error processing file: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
