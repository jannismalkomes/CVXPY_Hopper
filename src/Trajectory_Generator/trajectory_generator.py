"""
GFOLD (Guidance for Fuel-Optimal Large Diverts) Trajectory Generator

This module implements the GFOLD algorithm for spacecraft trajectory optimization,
specifically designed for powered descent guidance. Based on the convex optimization
approach presented in the GFOLD paper.

The implementation includes:
- Problem 3: Minimum Landing Error (with roughly solved final time)
- Problem 4: Minimum Fuel Use (with fixed landing point from P3)

Author: Converted from ARCHIVED_GFOLD_Static_py3.py
Style: Following CVXPYgen code generation example pattern
"""

import cvxpy as cp
import numpy as np
import time
import sys
import os
from typing import Tuple, Optional, Dict, Any
from scipy import signal
from scipy.interpolate import interp1d

# Optional imports for data export
try:
    import pandas as pd
    HAS_PANDAS = True
except ImportError:
    HAS_PANDAS = False
    print("Warning: pandas not available. CSV export will use basic numpy functionality.")


class HopperParameters:
    """
    Physical and operational parameters for the hopper spacecraft.
    Contains all the constants needed for the GFOLD optimization problem.
    """

    def __init__(self):
        # Physical constants
        self.g0 = 9.80665                    # Standard gravity [m/s²]
        self.m_dry = 140                     # Dry mass [kg]
        self.m_fuel = 60                     # Fuel mass [kg]
        self.m_wet = self.m_dry + self.m_fuel  # Total wet mass [kg]

        # Propulsion system parameters
        self.T_max = 2000                    # Maximum thrust [N]
        self.throt = [0.1, 1.0]             # Throttle range [min, max]
        self.Isp = 203.94                   # Specific impulse [s]
        self.alpha = 1 / (self.Isp * self.g0)  # Fuel consumption parameter
        self.r1 = self.throt[0] * self.T_max  # Lower thrust bound [N]
        self.r2 = self.throt[1] * self.T_max  # Upper thrust bound [N]

        # Operational constraints
        # Maximum structural acceleration [g]
        self.G_max = 10
        self.V_max = 90                      # Maximum velocity [m/s]
        self.y_gs = np.radians(30)          # Glide slope cone angle [rad]
        self.p_cs = np.radians(45)          # Thrust pointing constraint [rad]

        # Environmental parameters (Mars-like)
        self.g = np.array([-3.71, 0, 0])    # Gravity vector [m/s²]
        # Planetary angular velocity [rad/s]
        self.w = np.array([2.53e-5, 0, 6.62e-5])

        # Mission parameters
        self.r_initial = np.array([20, 5, 5])    # Initial position [m]
        self.v_initial = np.array([0, 0, 0])     # Initial velocity [m/s]
        self.r_target = np.array([0, 0, 0])      # Target landing position [m]
        # Target landing velocity [m/s]
        self.v_target = np.array([0, 0, 0])

        # Derived parameters
        self._compute_derived_parameters()

    def _compute_derived_parameters(self):
        """Compute derived parameters from basic physical constants."""
        # Basis vectors
        self.e = lambda i: signal.unit_impulse(3, i)

        # Skew-symmetric matrix for cross product
        def S(w_vec):
            return np.array([[0, -w_vec[2], w_vec[1]],
                             [w_vec[2], 0, -w_vec[0]],
                             [-w_vec[1], w_vec[0], 0]])

        # Glide slope constraint vector
        self.c = self.e(0) / np.tan(self.y_gs)

        # System matrices for dynamics
        self.A = np.zeros((6, 6))
        # Position-velocity coupling
        self.A[0:3, 3:6] = np.eye(3)
        # Centrifugal acceleration
        self.A[3:6, 0:3] = -np.square(S(self.w))
        self.A[3:6, 3:6] = -S(self.w)                  # Coriolis acceleration

        # Control input matrix
        self.B = np.array([[0, 0, 0],
                          [0, 0, 0],
                          [0, 0, 0],
                          [1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]])


class GFOLDTrajectoryGenerator:
    """
    GFOLD Trajectory Generator implementing convex optimization for spacecraft landing.

    This class encapsulates the GFOLD algorithm which solves two sequential problems:
    1. Problem 3: Minimize landing error for a given flight time
    2. Problem 4: Minimize fuel consumption with the landing point from Problem 3
    """

    def __init__(self, params: Optional[HopperParameters] = None):
        """
        Initialize the trajectory generator with given parameters.

        Args:
            params: HopperParameters object. If None, default parameters are used.
        """
        self.params = params if params is not None else HopperParameters()
        self.N = 120  # Number of discretization points
        self.dt = 0.1  # Time step [s]

        # Results storage
        self.last_solution = None
        self.last_problem_info = {}

    def _create_optimization_variables(self) -> Tuple[cp.Variable, cp.Variable, cp.Variable, cp.Variable]:
        """
        Create the optimization variables for the GFOLD problem.

        Returns:
            Tuple of (state, control, log_mass, thrust_slack) variables
        """
        x = cp.Variable(
            (6, self.N), name='state')      # State vector [position; velocity]
        # Control thrust vector normalized by mass
        u = cp.Variable((3, self.N), name='control')
        z = cp.Variable((1, self.N), name='log_mass')   # Logarithm of mass
        # Thrust slack parameter
        s = cp.Variable((1, self.N), name='thrust_slack')

        return x, u, z, s

    def _create_base_constraints(self, x: cp.Variable, u: cp.Variable,
                                 z: cp.Variable, s: cp.Variable,
                                 r0: np.ndarray) -> list:
        """
        Create the base constraints that are common to both Problem 3 and 4.

        Args:
            x: State variable
            u: Control variable  
            z: Log mass variable
            s: Thrust slack variable
            r0: Initial position

        Returns:
            List of constraint objects
        """
        constraints = []

        # Initial state conditions
        x0 = np.concatenate([r0, self.params.v_initial])
        constraints += [x[0:3, 0] == x0[0:3]]  # Initial position
        constraints += [x[3:6, 0] == x0[3:6]]  # Initial velocity

        # Final conditions
        constraints += [x[3:6, self.N-1] ==
                        self.params.v_target]  # Final velocity
        constraints += [s[0, self.N-1] == 0]  # No thrust at landing

        # Initial and final thrust direction constraints
        # Initial thrust direction
        constraints += [u[:, 0] == s[0, 0] * np.array([1, 0, 0])]
        constraints += [u[:, self.N-1] == s[0, self.N-1] *
                        np.array([1, 0, 0])]  # Final thrust direction

        # Initial mass constraint
        constraints += [z[0, 0] == cp.log(self.params.m_wet)]

        # Dynamic constraints for each time step
        for n in range(self.N - 1):
            # Leapfrog integration for dynamics
            constraints += [x[3:6, n+1] == x[3:6, n] +
                            (self.dt / 2) * ((u[:, n] + self.params.g) + (u[:, n+1] + self.params.g))]
            constraints += [x[0:3, n+1] == x[0:3, n] +
                            (self.dt / 2) * (x[3:6, n+1] + x[3:6, n])]

            # Glideslope constraint (keep within landing cone)
            pos_diff = x[0:3, n] - self.params.r_target
            constraints += [cp.norm(pos_diff[1:3]) <=
                            self.params.c[0] * (x[0, n] - self.params.r_target[0])]

            # Velocity magnitude constraint
            constraints += [cp.norm(x[3:6, n]) <= self.params.V_max]

            # Mass decrease due to fuel consumption
            constraints += [z[0, n+1] == z[0, n] -
                            (self.params.alpha * self.dt / 2) * (s[0, n] + s[0, n+1])]

            # Thrust magnitude constraint
            constraints += [cp.norm(u[:, n]) <= s[0, n]]

            # Thrust pointing constraint (thrust vector within cone)
            constraints += [u[0, n] >= np.cos(self.params.p_cs) * s[0, n]]

            # Thrust bounds (convex approximation of mass-normalized thrust)
            if n > 0:
                z0_term = self.params.m_wet - self.params.alpha * self.params.r2 * n * self.dt
                z1_term = self.params.m_wet - self.params.alpha * self.params.r1 * n * self.dt
                z0 = np.log(z0_term)
                z1 = np.log(z1_term)
                mu_2 = self.params.r2 / z0_term

                constraints += [s[0, n] <= mu_2 * (1 - (z[0, n] - z0))]
                constraints += [z[0, n] >= z0]
                constraints += [z[0, n] <= z1]

        # Altitude constraint (stay above surface except at landing)
        constraints += [x[0, 0:self.N-1] >= 0]

        return constraints

    def solve_problem_3(self, tf: float, r0: np.ndarray) -> Dict[str, Any]:
        """
        Solve Problem 3: Minimum Landing Error

        MINIMIZE: norm of landing error vector
        SUBJECT TO: all base constraints + final altitude constraint

        Args:
            tf: Flight time [s]
            r0: Initial position [m]

        Returns:
            Dictionary containing solution results and metadata
        """
        print("Solving Problem 3: Minimum Landing Error")
        print(f"Flight time: {tf:.2f} s")
        print(f"Initial position: {r0}")

        # Create optimization variables
        x, u, z, s = self._create_optimization_variables()

        # Create base constraints
        constraints = self._create_base_constraints(x, u, z, s, r0)

        # Add Problem 3 specific constraints
        constraints += [x[0, self.N-1] == 0]  # Final altitude must be zero

        # Define objective: minimize landing error
        objective = cp.Minimize(
            cp.norm(x[0:3, self.N-1] - self.params.r_target))

        # Create and solve problem
        problem = cp.Problem(objective, constraints)

        start_time = time.time()
        obj_value = problem.solve(solver=cp.ECOS, verbose=True, feastol=5e-20)
        solve_time = time.time() - start_time

        # Package results
        result = {
            'status': problem.status,
            'objective_value': obj_value,
            'solve_time': solve_time,
            'flight_time': tf,
            'variables': {
                'state': x.value if x.value is not None else None,
                'control': u.value if u.value is not None else None,
                'log_mass': z.value if z.value is not None else None,
                'thrust_slack': s.value if s.value is not None else None
            }
        }

        if x.value is not None:
            # Compute mass trajectory
            mass_trajectory = np.exp(z.value[0, :])
            result['mass_trajectory'] = mass_trajectory
            result['final_position'] = x.value[0:3, -1]
            result['final_velocity'] = x.value[3:6, -1]

            print(f"Problem 3 solved successfully!")
            print(f"Objective value (landing error): {obj_value:.6f}")
            print(f"Final position: {x.value[0:3, -1]}")
            print(f"Solve time: {solve_time:.3f} s")
        else:
            print("Problem 3 failed to solve!")

        self.last_solution = result
        return result

    def solve_problem_4(self, tf: float, r0: np.ndarray, rf_target: np.ndarray) -> Dict[str, Any]:
        """
        Solve Problem 4: Minimum Fuel Use

        MAXIMIZE: landing mass (minimize fuel consumption)
        SUBJECT TO: all base constraints + fixed landing point from Problem 3

        Args:
            tf: Flight time [s] 
            r0: Initial position [m]
            rf_target: Target final position from Problem 3 [m]

        Returns:
            Dictionary containing solution results and metadata
        """
        print("Solving Problem 4: Minimum Fuel Use")
        print(f"Flight time: {tf:.2f} s")
        print(f"Initial position: {r0}")
        print(f"Target final position: {rf_target}")

        # Create optimization variables
        x, u, z, s = self._create_optimization_variables()

        # Create base constraints
        constraints = self._create_base_constraints(x, u, z, s, r0)

        # Add Problem 4 specific constraints
        # Fixed final position from Problem 3
        constraints += [x[0:3, self.N-1] == rf_target]

        # Define objective: maximize final mass (minimize fuel consumption)
        objective = cp.Maximize(z[0, self.N-1])

        # Create and solve problem
        problem = cp.Problem(objective, constraints)

        start_time = time.time()
        obj_value = problem.solve(solver=cp.ECOS, verbose=True)
        solve_time = time.time() - start_time

        # Package results
        result = {
            'status': problem.status,
            'objective_value': obj_value,
            'solve_time': solve_time,
            'flight_time': tf,
            'variables': {
                'state': x.value if x.value is not None else None,
                'control': u.value if u.value is not None else None,
                'log_mass': z.value if z.value is not None else None,
                'thrust_slack': s.value if s.value is not None else None
            }
        }

        if x.value is not None:
            # Compute mass trajectory
            mass_trajectory = np.exp(z.value[0, :])
            result['mass_trajectory'] = mass_trajectory
            result['final_position'] = x.value[0:3, -1]
            result['final_velocity'] = x.value[3:6, -1]
            result['final_mass'] = mass_trajectory[-1]
            result['fuel_consumed'] = self.params.m_wet - mass_trajectory[-1]

            print(f"Problem 4 solved successfully!")
            print(f"Objective value (log final mass): {obj_value:.6f}")
            print(f"Final mass: {mass_trajectory[-1]:.2f} kg")
            print(f"Fuel consumed: {result['fuel_consumed']:.2f} kg")
            print(f"Final position: {x.value[0:3, -1]}")
            print(f"Solve time: {solve_time:.3f} s")
        else:
            print("Problem 4 failed to solve!")

        self.last_solution = result
        return result

    def solve_gfold_sequence(self, r0: Optional[np.ndarray] = None,
                             tf_estimate: float = 60.0) -> Dict[str, Any]:
        """
        Solve the complete GFOLD sequence (Problem 3 followed by Problem 4).

        This method implements the full GFOLD algorithm:
        1. Solve Problem 3 to find the optimal landing point with minimum error
        2. Use the result from Problem 3 to solve Problem 4 for minimum fuel consumption

        Args:
            r0: Initial position [m]. If None, uses default from parameters.
            tf_estimate: Estimated flight time [s]

        Returns:
            Dictionary containing results from both problems
        """
        if r0 is None:
            r0 = self.params.r_initial

        print("=" * 60)
        print("Starting GFOLD Sequential Solution")
        print("=" * 60)

        overall_start = time.time()

        # Step 1: Solve Problem 3 (Minimum Landing Error)
        p3_result = self.solve_problem_3(tf_estimate, r0)

        if p3_result['status'] not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            print(f"Problem 3 failed with status: {p3_result['status']}")
            return {'problem_3': p3_result, 'problem_4': None, 'success': False}

        # Extract landing point from Problem 3
        rf_from_p3 = p3_result['final_position']

        print("\n" + "=" * 60)

        # Step 2: Solve Problem 4 (Minimum Fuel Use)
        p4_result = self.solve_problem_4(tf_estimate, r0, rf_from_p3)

        if p4_result['status'] not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            print(f"Problem 4 failed with status: {p4_result['status']}")
            return {'problem_3': p3_result, 'problem_4': p4_result, 'success': False}

        overall_time = time.time() - overall_start

        print("\n" + "=" * 60)
        print("GFOLD Sequential Solution Complete!")
        print(f"Total solve time: {overall_time:.3f} s")
        print(f"Landing error from P3: {p3_result['objective_value']:.6f}")
        print(f"Fuel consumption from P4: {p4_result['fuel_consumed']:.2f} kg")
        print("=" * 60)

        return {
            'problem_3': p3_result,
            'problem_4': p4_result,
            'success': True,
            'total_solve_time': overall_time
        }

    def export_trajectory_data(self, filename: str = "trajectory.csv") -> bool:
        """
        Export trajectory data to CSV format for analysis and plotting.

        Args:
            filename: Output filename for CSV data

        Returns:
            True if export successful, False otherwise
        """
        if self.last_solution is None or self.last_solution['variables']['state'] is None:
            print("No solution data available for export!")
            return False

        try:
            # Extract trajectory data
            x = self.last_solution['variables']['state']
            time_vec = np.linspace(
                0, self.last_solution['flight_time'], self.N)

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
            t_new = np.arange(0, self.last_solution['flight_time'], 0.01)
            interpolated_data = {'time': t_new}

            for key in ['pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z']:
                f = interp1d(time_vec, trajectory_data[key], kind='linear')
                interpolated_data[key] = f(t_new)

            # Create directory if it doesn't exist
            os.makedirs('database/trajectories', exist_ok=True)

            # Save trajectory data
            if HAS_PANDAS:
                # Use pandas for cleaner CSV output
                df_interp = pd.DataFrame(interpolated_data)
                df_interp.to_csv(
                    f'database/trajectories/{filename}', index=False)
            else:
                # Use numpy for basic CSV output
                header = "time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z"
                data_array = np.column_stack([
                    interpolated_data['time'],
                    interpolated_data['pos_x'],
                    interpolated_data['pos_y'],
                    interpolated_data['pos_z'],
                    interpolated_data['vel_x'],
                    interpolated_data['vel_y'],
                    interpolated_data['vel_z']
                ])
                np.savetxt(f'database/trajectories/{filename}', data_array,
                           delimiter=',', header=header, comments='')

            print(
                f"Trajectory data exported to database/trajectories/{filename}")
            return True

        except Exception as e:
            print(f"Error exporting trajectory data: {e}")
            return False


if __name__ == "__main__":
    """
    Example usage of the GFOLD Trajectory Generator.

    This section demonstrates how to:
    1. Create a trajectory generator with default parameters
    2. Solve the complete GFOLD sequence (Problems 3 and 4)
    3. Extract and analyze results
    4. Export trajectory data
    """

    print("=" * 80)
    print("              GFOLD Trajectory Generator Example")
    print("=" * 80)

    # Change to script directory for consistent file operations
    script_dir = os.path.dirname(os.path.abspath(__file__))
    original_cwd = os.getcwd()
    os.chdir(script_dir)

    try:
        # Create trajectory generator with default hopper parameters
        generator = GFOLDTrajectoryGenerator()

        print(f"Hopper Parameters:")
        print(f"  Dry mass: {generator.params.m_dry} kg")
        print(f"  Fuel mass: {generator.params.m_fuel} kg")
        print(f"  Wet mass: {generator.params.m_wet} kg")
        print(f"  Maximum thrust: {generator.params.T_max} N")
        print(
            f"  Throttle range: {generator.params.throt[0]:.1f} - {generator.params.throt[1]:.1f}")
        print(f"  Specific impulse: {generator.params.Isp:.1f} s")
        print(f"  Initial position: {generator.params.r_initial} m")
        print(f"  Target position: {generator.params.r_target} m")
        print(f"  Maximum velocity: {generator.params.V_max} m/s")
        print()

        # Estimate flight time bounds
        tf_min = generator.params.m_dry * \
            np.linalg.norm(generator.params.v_initial) / generator.params.r2
        tf_max = (generator.params.m_wet - generator.params.m_dry) / \
            (generator.params.alpha * generator.params.r1)

        print(f"Flight time bounds:")
        print(f"  Minimum tf: {tf_min:.2f} s")
        print(f"  Maximum tf: {tf_max:.2f} s")
        print(f"  Using estimate: 60.0 s")
        print()

        # Solve complete GFOLD sequence
        results = generator.solve_gfold_sequence(tf_estimate=60.0)

        if results['success']:
            print("\nSUCCESS: GFOLD trajectory optimization completed!")

            # Display key results
            p3_result = results['problem_3']
            p4_result = results['problem_4']

            print(f"\nKey Results:")
            print(
                f"  Landing error (P3): {p3_result['objective_value']:.6f} m")
            print(f"  Final mass (P4): {p4_result['final_mass']:.2f} kg")
            print(f"  Fuel consumed: {p4_result['fuel_consumed']:.2f} kg")
            print(
                f"  Fuel efficiency: {(1 - p4_result['fuel_consumed']/generator.params.m_fuel)*100:.1f}%")
            print(
                f"  Total computation time: {results['total_solve_time']:.3f} s")

            # Extract final trajectory for display
            final_pos = p4_result['final_position']
            final_vel = p4_result['final_velocity']

            print(f"\nFinal State:")
            print(
                f"  Position: [{final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f}] m")
            print(
                f"  Velocity: [{final_vel[0]:.3f}, {final_vel[1]:.3f}, {final_vel[2]:.3f}] m/s")

            # Export trajectory data
            print(f"\nExporting trajectory data...")
            if generator.export_trajectory_data("gfold_trajectory.csv"):
                print(f"✓ Trajectory data exported successfully")
            else:
                print(f"✗ Failed to export trajectory data")

        else:
            print("\nFAILED: GFOLD trajectory optimization failed!")
            if results['problem_3']['status'] not in ['optimal', 'optimal_inaccurate']:
                print(f"Problem 3 status: {results['problem_3']['status']}")
            if results['problem_4'] and results['problem_4']['status'] not in ['optimal', 'optimal_inaccurate']:
                print(f"Problem 4 status: {results['problem_4']['status']}")

    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Restore original working directory
        os.chdir(original_cwd)

    print("\n" + "=" * 80)
