"""
GFOLD Data Exporter

This module provides comprehensive data extraction and export functionality for GFOLD
trajectory optimization results. It handles exporting solver statistics, trajectory data,
and optimization variables to various CSV formats for analysis and visualization.

The DataExporter class provides:
- Solver statistics extraction from CVXPY problems
- Basic trajectory data export
- Comprehensive optimization data export with multiple file formats
- High-resolution interpolated data generation
- Simplified position/velocity/acceleration exports

Dependencies:
- cvxpy: For accessing solver statistics and problem information
- numpy: Numerical computing
- pandas: Data manipulation and CSV export
- scipy: Interpolation functionality
- os: File system operations

Author: Extracted from trajectory_generator.py for modularity
"""

import cvxpy as cp
import numpy as np
import os
from typing import Dict, Any, Optional, Tuple
from scipy.interpolate import interp1d
import pandas as pd


class DataExporter:
    """
    Data extraction and export utility for GFOLD trajectory optimization results.

    This class provides comprehensive functionality to extract solver statistics,
    trajectory data, and optimization variables from GFOLD solutions and export
    them to various CSV formats for analysis and visualization.
    """

    def __init__(self, hopper_params, output_dir: str = "results/database"):
        """
        Initialize the DataExporter.

        Args:
            hopper_params: HopperParameters instance containing physical constants
            output_dir: Directory for data export files
        """
        self.params = hopper_params
        self.output_dir = output_dir

        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)

    def extract_solver_stats(self, problem: cp.Problem) -> Dict[str, Any]:
        """
        Extract comprehensive solver statistics and optimization results.

        Args:
            problem: Solved CVXPY problem instance

        Returns:
            Dictionary containing all available solver statistics
        """
        stats = {}

        # Basic problem information
        stats['problem_status'] = problem.status
        stats['optimal_value'] = problem.value
        stats['is_dcp'] = problem.is_dcp()
        stats['is_dpp'] = problem.is_dpp()
        stats['is_dgp'] = problem.is_dgp()
        stats['is_qp'] = problem.is_qp()

        # Problem size information
        try:
            size_metrics = problem.size_metrics
            stats['num_variables'] = getattr(size_metrics, 'num_scalar_variables',
                                             getattr(size_metrics, 'num_scalar_vars', None))
            stats['num_constraints'] = getattr(size_metrics, 'num_scalar_constraints',
                                               getattr(size_metrics, 'num_scalar_constr', None))
            stats['num_parameters'] = getattr(size_metrics, 'num_scalar_parameters',
                                              getattr(size_metrics, 'num_scalar_params', None))

            # Add all available size metrics
            for attr in dir(size_metrics):
                if not attr.startswith('_') and not callable(getattr(size_metrics, attr)):
                    try:
                        stats[f'size_{attr}'] = getattr(size_metrics, attr)
                    except:
                        pass
        except Exception as e:
            stats['size_metrics_error'] = str(e)

        # Solver statistics (if available)
        if hasattr(problem, 'solver_stats') and problem.solver_stats:
            solver_stats = problem.solver_stats
            stats['solver_name'] = solver_stats.solver_name if hasattr(
                solver_stats, 'solver_name') else 'Unknown'
            stats['num_iters'] = solver_stats.num_iters if hasattr(
                solver_stats, 'num_iters') else None
            stats['solve_time'] = solver_stats.solve_time if hasattr(
                solver_stats, 'solve_time') else None
            stats['setup_time'] = solver_stats.setup_time if hasattr(
                solver_stats, 'setup_time') else None

            # ECOS specific statistics
            if hasattr(solver_stats, 'pcost'):
                stats['primal_cost'] = solver_stats.pcost
            if hasattr(solver_stats, 'dcost'):
                stats['dual_cost'] = solver_stats.dcost
            if hasattr(solver_stats, 'gap'):
                stats['duality_gap'] = solver_stats.gap
            if hasattr(solver_stats, 'pres'):
                stats['primal_residual'] = solver_stats.pres
            if hasattr(solver_stats, 'dres'):
                stats['dual_residual'] = solver_stats.dres
            if hasattr(solver_stats, 'exitflag'):
                stats['exit_flag'] = solver_stats.exitflag
            if hasattr(solver_stats, 'infostring'):
                stats['info_string'] = solver_stats.infostring

        # Extract all available attributes from solver_stats
        if hasattr(problem, 'solver_stats') and problem.solver_stats:
            for attr in dir(problem.solver_stats):
                if not attr.startswith('_') and attr not in ['solver_name', 'num_iters', 'solve_time', 'setup_time']:
                    try:
                        value = getattr(problem.solver_stats, attr)
                        if not callable(value):
                            stats[f'solver_{attr}'] = value
                    except:
                        pass

        return stats

    def export_trajectory_data(self, solution_data: Dict[str, Any], N: int,
                               filename: str = "trajectory.csv") -> bool:
        """
        Export trajectory data to CSV format for analysis and plotting.

        Args:
            solution_data: Solution dictionary containing trajectory variables
            N: Number of time steps
            filename: Output filename for CSV data

        Returns:
            True if export successful, False otherwise
        """
        if solution_data is None or solution_data.get('variables', {}).get('state') is None:
            print("No solution data available for export!")
            return False

        try:
            # Extract trajectory data
            x = solution_data['variables']['state']
            time_vec = np.linspace(0, solution_data['flight_time'], N)

            # Create DataFrame with trajectory data
            trajectory_data = {
                'time': time_vec,
                'pos_x': x[0, :],  # Altitude
                'pos_y': x[1, :],  # Cross-range
                'pos_z': x[2, :],  # Down-range
                'vel_x': x[3, :],  # Altitude velocity
                'vel_y': x[4, :],  # Cross-range velocity
                'vel_z': x[5, :]   # Down-range velocity
            }

            # Create interpolated data at higher resolution (0.01s intervals)
            t_new = np.arange(0, solution_data['flight_time'], 0.01)
            interpolated_data = {'time': t_new}

            for key in ['pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z']:
                f = interp1d(time_vec, trajectory_data[key], kind='linear')
                interpolated_data[key] = f(t_new)

            # Save trajectory data using pandas
            df_interp = pd.DataFrame(interpolated_data)
            df_interp.to_csv(f'{self.output_dir}/{filename}', index=False)

            print(f"Trajectory data exported to {self.output_dir}/{filename}")
            return True

        except Exception as e:
            print(f"Error exporting trajectory data: {e}")
            return False

    def export_comprehensive_data(self, solution_data: Dict[str, Any], N: int,
                                  problem3_stats: Optional[Dict[str, Any]] = None,
                                  problem4_stats: Optional[Dict[str, Any]] = None,
                                  filename_base: str = "gfold_comprehensive") -> bool:
        """
        Export comprehensive optimization data including all solver statistics,
        trajectory data, and optimization variables.

        Args:
            solution_data: Solution dictionary containing all trajectory variables
            N: Number of time steps
            problem3_stats: Problem 3 solver statistics (optional)
            problem4_stats: Problem 4 solver statistics (optional)
            filename_base: Base filename for exports (will create multiple files)

        Returns:
            True if export successful, False otherwise
        """
        if solution_data is None:
            print("No solution data available for comprehensive export!")
            return False

        try:
            # 1. Export solver statistics and optimization metadata
            optimization_data = {}

            # Problem 3 data (if available)
            if problem3_stats:
                optimization_data['problem3'] = problem3_stats

            # Problem 4 data (if available)
            if problem4_stats:
                optimization_data['problem4'] = problem4_stats

            # General optimization information
            optimization_data['general'] = {
                'algorithm': 'GFOLD Sequential (Problem 3 + Problem 4)',
                'N_time_steps': N,
                'flight_time': solution_data['flight_time'],
                'total_solve_time': solution_data.get('total_solve_time', 0),
                'fuel_consumed_kg': (self.params.m_wet - np.exp(solution_data['variables']['log_mass'][0, -1])) if solution_data['variables']['log_mass'] is not None else None,
                'final_mass_kg': np.exp(solution_data['variables']['log_mass'][0, -1]) if solution_data['variables']['log_mass'] is not None else None,
                'landing_error_m': np.linalg.norm(solution_data['final_position']) if 'final_position' in solution_data else None,
                'final_velocity_magnitude': np.linalg.norm(solution_data['final_velocity']) if 'final_velocity' in solution_data else None
            }

            # Export optimization metadata to JSON-like CSV
            opt_df = pd.json_normalize(optimization_data, sep='_')
            opt_df.to_csv(
                f'{self.output_dir}/{filename_base}_optimization_stats.csv', index=False)

            # 2. Export detailed trajectory data
            if solution_data['variables']['state'] is not None:
                x = solution_data['variables']['state']
                u = solution_data['variables']['control']
                z = solution_data['variables']['log_mass']
                s = solution_data['variables']['thrust_slack']
                time_vec = np.linspace(0, solution_data['flight_time'], N)

                # Comprehensive trajectory data
                detailed_data = {
                    'time': time_vec,
                    'pos_x_altitude': x[0, :],
                    'pos_y_crossrange': x[1, :],
                    'pos_z_downrange': x[2, :],
                    'vel_x_altitude': x[3, :],
                    'vel_y_crossrange': x[4, :],
                    'vel_z_downrange': x[5, :],
                    'control_x': u[0, :] if u is not None else np.zeros(N),
                    'control_y': u[1, :] if u is not None else np.zeros(N),
                    'control_z': u[2, :] if u is not None else np.zeros(N),
                    'log_mass': z[0, :] if z is not None else np.zeros(N),
                    'mass_kg': np.exp(z[0, :]) if z is not None else np.zeros(N),
                    'thrust_slack': s[0, :] if s is not None else np.zeros(N),
                    'thrust_magnitude': np.linalg.norm(u, axis=0) if u is not None else np.zeros(N),
                    'velocity_magnitude': np.linalg.norm(x[3:6, :], axis=0),
                    'position_magnitude': np.linalg.norm(x[0:3, :], axis=0),
                    'fuel_consumed_kg': self.params.m_wet - (np.exp(z[0, :]) if z is not None else self.params.m_wet),
                    'throttle_percentage': (np.linalg.norm(u, axis=0) / self.params.T_max * 100) if u is not None else np.zeros(N)
                }

                # Add acceleration data (u represents thrust acceleration in m/s²)
                if u is not None:
                    # Thrust acceleration components (from optimization variable u)
                    # Thrust acceleration X (altitude)
                    detailed_data['accel_thrust_x'] = u[0, :]
                    # Thrust acceleration Y (cross-range)
                    detailed_data['accel_thrust_y'] = u[1, :]
                    # Thrust acceleration Z (down-range)
                    detailed_data['accel_thrust_z'] = u[2, :]
                    detailed_data['accel_thrust_magnitude'] = np.linalg.norm(
                        u, axis=0)

                    # Total acceleration (thrust + gravity)
                    detailed_data['accel_total_x'] = u[0, :] + \
                        self.params.g[0]  # Total acceleration X
                    detailed_data['accel_total_y'] = u[1, :] + \
                        self.params.g[1]  # Total acceleration Y
                    detailed_data['accel_total_z'] = u[2, :] + \
                        self.params.g[2]  # Total acceleration Z
                    detailed_data['accel_total_magnitude'] = np.linalg.norm(
                        u + self.params.g.reshape(-1, 1), axis=0)

                    # Gravitational acceleration components (constant)
                    detailed_data['accel_gravity_x'] = np.full(
                        N, self.params.g[0])
                    detailed_data['accel_gravity_y'] = np.full(
                        N, self.params.g[1])
                    detailed_data['accel_gravity_z'] = np.full(
                        N, self.params.g[2])
                    detailed_data['accel_gravity_magnitude'] = np.full(
                        N, np.linalg.norm(self.params.g))

                    # Acceleration in g-forces for engineering analysis
                    g_magnitude = np.linalg.norm(self.params.g)
                    detailed_data['accel_thrust_gforce'] = np.linalg.norm(
                        u, axis=0) / g_magnitude
                    detailed_data['accel_total_gforce'] = np.linalg.norm(
                        u + self.params.g.reshape(-1, 1), axis=0) / g_magnitude
                else:
                    # Zero acceleration if no control data
                    for accel_key in ['accel_thrust_x', 'accel_thrust_y', 'accel_thrust_z', 'accel_thrust_magnitude',
                                      'accel_total_x', 'accel_total_y', 'accel_total_z', 'accel_total_magnitude',
                                      'accel_gravity_x', 'accel_gravity_y', 'accel_gravity_z', 'accel_gravity_magnitude',
                                      'accel_thrust_gforce', 'accel_total_gforce']:
                        detailed_data[accel_key] = np.zeros(N)

                # Create high-resolution interpolated data
                t_highres = np.arange(0, solution_data['flight_time'], 0.01)
                interp_data = {'time': t_highres}

                for key, values in detailed_data.items():
                    if key != 'time':
                        f = interp1d(time_vec, values, kind='linear',
                                     bounds_error=False, fill_value='extrapolate')
                        interp_data[key] = f(t_highres)

                # Export detailed trajectory data
                detailed_df = pd.DataFrame(detailed_data)
                detailed_df.to_csv(
                    f'{self.output_dir}/{filename_base}_detailed_trajectory.csv', index=False)

                # Export high-resolution interpolated data
                interp_df = pd.DataFrame(interp_data)
                interp_df.to_csv(
                    f'{self.output_dir}/{filename_base}_interpolated_trajectory.csv', index=False)

                # Export simplified trajectory data (position, velocity, acceleration only)
                if u is not None:
                    simplified_data = {
                        'time': time_vec,
                        'pos_x': x[0, :],  # Position X (altitude)
                        'pos_y': x[1, :],  # Position Y (cross-range)
                        'pos_z': x[2, :],  # Position Z (down-range)
                        'vel_x': x[3, :],  # Velocity X (altitude)
                        'vel_y': x[4, :],  # Velocity Y (cross-range)
                        'vel_z': x[5, :],  # Velocity Z (down-range)
                        # Total acceleration X
                        'acc_x': u[0, :] + self.params.g[0],
                        # Total acceleration Y
                        'acc_y': u[1, :] + self.params.g[1],
                        # Total acceleration Z
                        'acc_z': u[2, :] + self.params.g[2]
                    }

                    # Create high-resolution simplified interpolated data
                    simplified_interp_data = {'time': t_highres}
                    for key, values in simplified_data.items():
                        if key != 'time':
                            f = interp1d(time_vec, values, kind='linear',
                                         bounds_error=False, fill_value='extrapolate')
                            simplified_interp_data[key] = f(t_highres)

                    # Export simplified trajectory data
                    simplified_df = pd.DataFrame(simplified_data)
                    simplified_df.to_csv(
                        f'{self.output_dir}/{filename_base}_simple_trajectory.csv', index=False)

                    # Export simplified high-resolution interpolated data
                    simplified_interp_df = pd.DataFrame(simplified_interp_data)
                    simplified_interp_df.to_csv(
                        f'{self.output_dir}/{filename_base}_simple_interpolated.csv', index=False)

            print(f"✓ Comprehensive optimization data exported:")
            print(
                f"  - {self.output_dir}/{filename_base}_optimization_stats.csv")
            print(
                f"  - {self.output_dir}/{filename_base}_detailed_trajectory.csv")
            print(
                f"  - {self.output_dir}/{filename_base}_interpolated_trajectory.csv")
            print(f"  - {self.output_dir}/{filename_base}_simple_trajectory.csv")
            print(
                f"  - {self.output_dir}/{filename_base}_simple_interpolated.csv")

            return True

        except Exception as e:
            print(f"Error exporting comprehensive data: {e}")
            return False

    def export_problem4_data(self, solution_data: Dict[str, Any],
                             filename_prefix: str = "problem4") -> bool:
        """
        Export all data outputs for Problem 4 (minimum fuel use) only.

        This is a single function that generates all necessary data exports
        when only Problem 4 has been executed, without requiring Problem 3 data.

        Args:
            solution_data: Dictionary containing Problem 4 solution results
            filename_prefix: Prefix for output filenames

        Returns:
            bool: True if export successful, False otherwise
        """
        try:
            print("\nExporting Problem 4 data...")

            # Ensure output directory exists
            os.makedirs(self.output_dir, exist_ok=True)

            # Extract solution variables
            x = solution_data['variables']['state']
            u = solution_data['variables']['control']
            z = solution_data['variables']['log_mass']
            s = solution_data['variables']['thrust_slack']

            if x is None or u is None:
                print("Error: No solution data available for export")
                return False

            # Get number of time points from solution data
            N = x.shape[1] if x is not None else u.shape[1]

            # Create time vector
            tf = solution_data['flight_time']
            time_vec = np.linspace(0, tf, N)

            # 1. Export basic trajectory data
            trajectory_data = {
                'time': time_vec,
                'pos_x': x[0, :], 'pos_y': x[1, :], 'pos_z': x[2, :],
                'vel_x': x[3, :], 'vel_y': x[4, :], 'vel_z': x[5, :],
                'thrust_x': u[0, :], 'thrust_y': u[1, :], 'thrust_z': u[2, :],
            }

            if z is not None:
                trajectory_data.update({
                    'log_mass': z[0, :],
                    'mass': np.exp(z[0, :])
                })

            if s is not None:
                trajectory_data['thrust_magnitude'] = s[0, :]

            basic_df = pd.DataFrame(trajectory_data)
            basic_filename = f'{self.output_dir}/{filename_prefix}_trajectory.csv'
            basic_df.to_csv(basic_filename, index=False)
            print(f"✓ Basic trajectory data: {basic_filename}")

            # 2. Export solver statistics
            if 'solver_stats' in solution_data:
                stats_data = solution_data['solver_stats']

                # Add problem-specific metadata
                stats_data.update({
                    'problem_type': 'Problem 4 - Minimum Fuel Use',
                    'flight_time': tf,
                    'initial_position_x': self.params.r_initial[0],
                    'initial_position_y': self.params.r_initial[1],
                    'initial_position_z': self.params.r_initial[2],
                    'final_position_x': x[0, -1],
                    'final_position_y': x[1, -1],
                    'final_position_z': x[2, -1],
                    'final_mass': solution_data.get('final_mass', np.exp(z[0, -1]) if z is not None else None),
                    'fuel_consumed': solution_data.get('fuel_consumed', None),
                    'fuel_efficiency': (solution_data.get('final_mass', np.exp(z[0, -1]) if z is not None else None) / self.params.m_wet * 100) if z is not None else None
                })

                stats_df = pd.DataFrame([stats_data])
                stats_filename = f'{self.output_dir}/{filename_prefix}_solver_stats.csv'
                stats_df.to_csv(stats_filename, index=False)
                print(f"✓ Solver statistics: {stats_filename}")

            # 3. Export detailed trajectory with computed quantities
            detailed_data = {
                'time': time_vec,
                'pos_x': x[0, :], 'pos_y': x[1, :], 'pos_z': x[2, :],
                'vel_x': x[3, :], 'vel_y': x[4, :], 'vel_z': x[5, :],
                'thrust_x': u[0, :], 'thrust_y': u[1, :], 'thrust_z': u[2, :],
                'position_magnitude': np.linalg.norm(x[0:3, :], axis=0),
                'velocity_magnitude': np.linalg.norm(x[3:6, :], axis=0),
                'thrust_magnitude': np.linalg.norm(u, axis=0),
                'acceleration_x': u[0, :] + self.params.g[0],
                'acceleration_y': u[1, :] + self.params.g[1],
                'acceleration_z': u[2, :] + self.params.g[2],
            }

            if z is not None:
                mass_trajectory = np.exp(z[0, :])
                detailed_data.update({
                    'log_mass': z[0, :],
                    'mass': mass_trajectory,
                    'thrust_to_weight': np.linalg.norm(u, axis=0) / (mass_trajectory * abs(self.params.g[0]))
                })

            detailed_df = pd.DataFrame(detailed_data)
            detailed_filename = f'{self.output_dir}/{filename_prefix}_detailed.csv'
            detailed_df.to_csv(detailed_filename, index=False)
            print(f"✓ Detailed trajectory: {detailed_filename}")

            # 4. Export high-resolution interpolated data
            n_interp = 1000
            t_highres = np.linspace(0, tf, n_interp)

            interp_data = {'time': t_highres}
            for key, values in detailed_data.items():
                if key != 'time':
                    f = interp1d(time_vec, values, kind='linear',
                                 bounds_error=False, fill_value='extrapolate')
                    interp_data[key] = f(t_highres)

            interp_df = pd.DataFrame(interp_data)
            interp_filename = f'{self.output_dir}/{filename_prefix}_interpolated.csv'
            interp_df.to_csv(interp_filename, index=False)
            print(f"✓ Interpolated trajectory: {interp_filename}")

            # 5. Export simplified trajectory (position, velocity, acceleration only)
            simplified_data = {
                'time': time_vec,
                'pos_x': x[0, :], 'pos_y': x[1, :], 'pos_z': x[2, :],
                'vel_x': x[3, :], 'vel_y': x[4, :], 'vel_z': x[5, :],
                'acc_x': u[0, :] + self.params.g[0],
                'acc_y': u[1, :] + self.params.g[1],
                'acc_z': u[2, :] + self.params.g[2]
            }

            simplified_df = pd.DataFrame(simplified_data)
            simplified_filename = f'{self.output_dir}/{filename_prefix}_simple.csv'
            simplified_df.to_csv(simplified_filename, index=False)
            print(f"✓ Simplified trajectory: {simplified_filename}")

            print(f"\n✓ Problem 4 data export completed successfully!")
            print(f"  Output directory: {self.output_dir}/")
            print(f"  Files generated:")
            print(f"    - {filename_prefix}_trajectory.csv (basic data)")
            print(
                f"    - {filename_prefix}_solver_stats.csv (optimization statistics)")
            print(
                f"    - {filename_prefix}_detailed.csv (comprehensive trajectory)")
            print(
                f"    - {filename_prefix}_interpolated.csv (high-resolution data)")
            print(
                f"    - {filename_prefix}_simple.csv (position/velocity/acceleration)")

            return True

        except Exception as e:
            print(f"Error exporting Problem 4 data: {e}")
            return False
