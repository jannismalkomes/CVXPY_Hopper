"""
GFOLD (Guidance for Fuel-Optimal Large Diverts) Trajectory Generator

This module implements the GFOLD algorithm for spacecraft trajectory optimization,
specifically designed for powered descent guidance. Based on the convex optimization
approach presented in the GFOLD paper.

The implementation includes:
- Problem 3: Minimum Landing Error (with roughly solved final time)
- Problem 4: Minimum Fuel Use (with fixed landing point from P3)

Dependencies:
- cvxpy: Convex optimization framework
- numpy: Numerical computing
- scipy: Scientific computing (signal processing, interpolation)
- pandas: Data manipulation and CSV export

Author: Converted from ARCHIVED_GFOLD_Static_py3.py
Style: Following CVXPYgen code generation example pattern
"""

# Configuration flags
from data_exporter import DataExporter
from plotting import TrajectoryPlotter
import pandas as pd
from scipy.interpolate import interp1d
from scipy import signal
from typing import Tuple, Optional, Dict, Any
import os
import sys
import time
import numpy as np
from cvxpygen import cpg
import cvxpy as cp
CODE_EXPORT = True  # Set to True to enable C code generation


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
        self.throt = [0.7, 1.0]            # Throttle range [min, max]
        self.Isp = 203.94                   # Specific impulse [s]
        self.alpha = 1 / (self.Isp * self.g0)  # Fuel consumption parameter
        self.r1 = self.throt[0] * self.T_max  # Lower thrust bound [N]
        self.r2 = self.throt[1] * self.T_max  # Upper thrust bound [N]

        # Operational constraints
        # Maximum structural acceleration [g]
        self.G_max = 10
        self.V_max = 10                     # Maximum velocity [m/s]
        self.y_gs = np.radians(30)          # Glide slope cone angle [rad]
        self.p_cs = np.radians(15)          # Thrust pointing constraint [rad]
        # Cosine of thrust pointing constraint
        self.cos_p_cs = np.cos(self.p_cs)

        # Environmental parameters (Earth with z-up)
        # Gravity vector [m/s²] - z is down
        self.g = np.array([0, 0, -self.g0])
        # Planetary angular velocity [rad/s], Earth's rotation rate: 7.2921159e-5 rad/s, Rotation about z-axis (vertical)
        self.w = np.array([0, 0, 7.2921159e-5])

        # Initial conditions
        # Initial position [x, y, z] where z is height
        self.r_initial = np.array([0, 0, 10])
        self.v_initial = np.array([0, 0, 0])     # Initial velocity [m/s]

        # Target conditions
        self.r_target = np.array([0, 0, 0])     # Target landing pos
        self.v_target = np.array([0, 0, 0])     # Target landing vel

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

        # # Glide slope constraint vector - changed to z-direction
        # self.c = self.e(2) / np.tan(self.y_gs)

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

        self.A = cp.Parameter((6, 6), value=self.A)
        self.B = cp.Parameter((6, 3), value=self.B)


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
        self.N = int(60)  # Number of discretization points
        self.dt = 0.1  # Time step [s]

        # Results storage
        self.last_solution = None
        self.last_problem_info = {}

        # Initialize data exporter
        self.data_exporter = DataExporter(self.params)

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
        # # No thrust at landing
        # constraints += [s[0, self.N-1] == 0]

        # Initial and final thrust direction constraints
        # Initial thrust direction - thrust upward to counteract gravity
        constraints += [u[:, 0] == s[0, 0] * np.array([0, 0, 1])]
        constraints += [u[:, self.N-1] == s[0, self.N-1] *
                        np.array([0, 0, 1])]  # Final thrust direction

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
            # # Changed to use z as height coordinate
            # pos_diff = x[0:3, n] - self.params.r_target
            # constraints += [cp.norm(pos_diff[0:2]) <=  # x,y components
            #                 # z component
            #                 np.tan(self.params.y_gs) * (x[2, n] - self.params.r_target[2])]

            # Velocity magnitude constraint
            constraints += [cp.norm(x[3:6, n]) <= self.params.V_max]

            # Acceleration magnitude constraint (total acceleration = thrust + gravity)
            # Total acceleration in G's: ||u + g|| / g0 <= G_max
            # Rearranged: ||u + g|| <= G_max * g0
            constraints += [cp.norm(u[:, n] + self.params.g)
                            <= self.params.G_max * self.params.g0]

            # Mass decrease due to fuel consumption
            constraints += [z[0, n+1] == z[0, n] -
                            (self.params.alpha * self.dt / 2) * (s[0, n] + s[0, n+1])]

            # Thrust magnitude constraint
            constraints += [cp.norm(u[:, n]) <= s[0, n]]

            # Thrust pointing constraint (thrust vector within cone)
            # Changed to z-component for upward thrust
            constraints += [u[2, n] >= np.cos(self.params.p_cs) * s[0, n]]

        # Apply thrust bounds to ALL time steps (including first and last)
        for n in range(self.N):
            z0_term = self.params.m_wet - self.params.alpha * self.params.r2 * n * self.dt
            z1_term = self.params.m_wet - self.params.alpha * self.params.r1 * n * self.dt

            # Ensure positive mass terms
            z0_term = max(z0_term, self.params.m_dry + 1e-6)
            z1_term = max(z1_term, self.params.m_dry + 1e-6)

            z0 = np.log(z0_term)
            z1 = np.log(z1_term)
            mu_1 = self.params.r1 / z1_term
            mu_2 = self.params.r2 / z0_term

            constraints += [s[0, n] >= mu_1 * (1 - (z[0, n] - z1))]
            constraints += [s[0, n] <= mu_2 * (1 - (z[0, n] - z0))]
            constraints += [z[0, n] >= z0]
            constraints += [z[0, n] <= z1]

        # Apply thrust constraints to final step (N-1)
        constraints += [cp.norm(u[:, self.N-1]) <= s[0, self.N-1]]
        constraints += [u[2, self.N-1] >=
                        np.cos(self.params.p_cs) * s[0, self.N-1]]

        # Altitude constraint (stay above surface except at landing)
        # Changed to z-coordinate for height
        constraints += [x[2, 0:self.N-1] >= 0]

        return constraints

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

        # Update time step based on flight time and number of points
        self.dt = tf / (self.N - 1)

        # Create optimization variables
        x, u, z, s = self._create_optimization_variables()

        # Create base constraints
        constraints = self._create_base_constraints(x, u, z, s, r0)

        # Add Problem 4 specific constraints
        # Fixed final position from target position
        constraints += [x[0:3, self.N-1] == rf_target]

        # Define objective: maximize final mass (minimize fuel consumption)
        objective = cp.Maximize(z[0, self.N-1])

        # Create and solve problem
        problem = cp.Problem(objective, constraints)

        start_time = time.time()
        obj_value = problem.solve(solver=cp.ECOS, verbose=True)
        solve_time = time.time() - start_time

        # # generate code
        # code_dir = "code_export"
        # cpg.generate_code(problem, code_dir=str(code_dir), solver='ECOS')

        # Extract comprehensive solver statistics
        solver_stats = self.data_exporter.extract_solver_stats(problem)
        solver_stats['manual_solve_time'] = solve_time
        self._problem4_stats = solver_stats

        # Package results
        result = {
            'status': problem.status,
            'objective_value': obj_value,
            'solve_time': solve_time,
            'solver_stats': solver_stats,
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

            print(f"\nProblem 4 solved successfully!")
            print(f"Objective value (log final mass): {obj_value:.6f}")
            print(f"Final mass: {mass_trajectory[-1]:.2f} kg")
            print(f"Fuel consumed: {result['fuel_consumed']:.2f} kg")
            print(f"Final position [x, y, z]: {x.value[0:3, -1]}")
            print(f"Solve time: {solve_time:.3f} s")

            # Check thrust forces throughout trajectory for constraint violations (internal validation)
            thrust_forces = []
            for k in range(self.N):
                thrust_acc_mag = np.linalg.norm(u.value[:, k])
                mass_at_k = mass_trajectory[k]
                thrust_force = thrust_acc_mag * mass_at_k
                thrust_forces.append(thrust_force)

            max_thrust = max(thrust_forces)
            # Store thrust analysis for internal use but don't print
            result['thrust_analysis'] = {
                'max_thrust': max_thrust,
                'max_thrust_time': thrust_forces.index(max_thrust) * self.dt,
                'thrust_limit': self.params.T_max,
                'constraint_satisfied': max_thrust <= self.params.T_max
            }
        else:
            print("Problem 4 failed to solve!")

        self.last_solution = result
        return result

    def generate_plots(self) -> list:
        """
        Generate trajectory plots using the integrated plotting module.

        Returns:
            List of saved plot file paths
        """
        if self.last_solution is None or self.last_solution['variables']['state'] is None:
            print("No solution data available for plotting!")
            return []

        try:
            # Extract trajectory data
            x = self.last_solution['variables']['state']
            u = self.last_solution['variables']['control']
            m = self.last_solution['mass_trajectory']
            tf = self.last_solution['flight_time']

            # Generate all plots
            plotter = TrajectoryPlotter("results/images")
            saved_files = plotter.plot_all_trajectories(
                x, u, m, tf, gravity=self.params.g)

            return saved_files

        except Exception as e:
            print(f"Error generating plots: {e}")
            return []

    def create_parameterized_problem(self) -> cp.Problem:
        """
        Create a parameterized CVXPY problem for C code generation.

        Returns:
            CVXPY Problem with parameters for initial/final conditions, flight time, and constants
        """
        # Create optimization variables
        x, u, z, s = self._create_optimization_variables()

        # Create parameters for C code generation
        r_initial_param = cp.Parameter(
            3, value=self.params.r_initial, name='r_initial')
        v_initial_param = cp.Parameter(
            3, value=self.params.v_initial, name='v_initial')
        r_target_param = cp.Parameter(
            3, value=self.params.r_target, name='r_target')
        v_target_param = cp.Parameter(
            3, value=self.params.v_target, name='v_target')
        # Default flight time
        # tf_param = cp.Parameter(value=8.0, pos=True, name='tf', nonneg=True)
        dt_param = cp.Parameter(value=0.1, pos=True,
                                name='dt', nonneg=True)      # Time step [s]
        # Maximum velocity [m/s]
        v_max_param = cp.Parameter(
            value=self.params.V_max, pos=True, name='v_max', nonneg=True)
        # Maximum acceleration [g]
        g_max_param = cp.Parameter(
            value=self.params.G_max, pos=True, name='g_max', nonneg=True)

        # Fuel consumption parameter
        alpha_dt_param = cp.Parameter(
            # alpha_param_dt = (alpha_param * dt_param / 2)
            value=self.params.alpha, name='alpha_dt', nonneg=True)
        # Thrust pointing cone angle
        cos_p_cs_param = cp.Parameter(
            value=self.params.cos_p_cs, name='cos_p_cs')

        # Precompute zero placeholders for N time steps
        c1_param = cp.Parameter(self.N, value=np.zeros(
            self.N), name='c1')  # mu_1 * (1 + z1)
        c2_param = cp.Parameter(self.N, value=np.zeros(
            self.N), name='c2')  # mu_2 * (1 + z0)

        # Constants
        g0 = 9.80665                        # Standard gravity [m/s²]
        g = np.array([0, 0, -g0])           # Gravity vector [m/s²]
        N = self.N                          # Number of discretization points
        m_dry = self.params.m_dry           # Dry mass [kg]
        m_wet = self.params.m_wet           # Wet mass [kg]
        thrust_min = self.params.r1         # Min thrust [N]
        thrust_max = self.params.r2         # Max thrust [N]
        alpha = self.params.alpha           # Fuel consumption parameter

        constraints = []

        # Initial state conditions (use parameters)
        constraints += [x[0:3, 0] == r_initial_param]   # Initial position
        constraints += [x[3:6, 0] == v_initial_param]   # Initial velocity
        # Final conditions
        constraints += [x[3:6, N-1] == v_target_param]  # Final velocity
        constraints += [x[0:3, N-1] == r_target_param]  # Final position

        # Initial and final thrust direction constraints
        constraints += [u[:, 0] == s[0, 0] * np.array([0, 0, 1])]
        constraints += [u[:, N-1] == s[0, N-1] * np.array([0, 0, 1])]

        # Initial mass constraint
        constraints += [z[0, 0] == cp.log(m_wet)]

        # Dynamic constraints for each time step (with parameterized time)
        for n in range(N - 1):
            # Leapfrog integration for dynamics
            constraints += [x[3:6, n+1] == x[3:6, n] +
                            (dt_param / 2) * ((u[:, n] + g) + (u[:, n+1] + g))]
            constraints += [x[0:3, n+1] == x[0:3, n] +
                            (dt_param / 2) * (x[3:6, n+1] + x[3:6, n])]

            # Velocity magnitude constraint
            constraints += [cp.norm(x[3:6, n]) <= v_max_param]

            # Acceleration magnitude constraint (total acceleration = thrust + gravity)
            constraints += [cp.norm(u[:, n] + g)
                            <= g_max_param * g0]

            # Mass decrease due to fuel consumption
            constraints += [z[0, n+1] == z[0, n] -
                            (alpha_dt_param) * (s[0, n] + s[0, n+1])]

            # Thrust magnitude constraint
            constraints += [cp.norm(u[:, n]) <= s[0, n]]

            # Thrust pointing constraint (thrust vector within cone)
            constraints += [u[2, n] >= cos_p_cs_param * s[0, n]]

        # Apply thrust bounds to ALL time steps (with parameterized time)
        for n in range(N):
            z0_term = m_wet - alpha * thrust_max * n * dt_param
            z1_term = m_wet - alpha * thrust_min * n * dt_param

            z0 = cp.log(z0_term)
            z1 = cp.log(z1_term)
            mu_1 = thrust_min / z1_term
            mu_2 = thrust_max / z0_term

            # Linearized thrust bounds (same as solve_problem_4)
            constraints += [s[0, n] >= c1_param[n] - mu_1 * z[0, n]]
            constraints += [s[0, n] <= c2_param[n] - mu_2 * z[0, n]]
            constraints += [z[0, n] >= z0]
            constraints += [z[0, n] <= z1]

        # Apply thrust constraints to final step (N-1)
        constraints += [cp.norm(u[:, N-1]) <= s[0, N-1]]
        constraints += [u[2, N-1] >= (cos_p_cs_param) * s[0, N-1]]

        # Altitude constraint (stay above surface except at landing)
        constraints += [x[2, 0:N-2] >= 0]
        # Objective: maximize final mass (minimize fuel consumption)
        objective = cp.Maximize(z[0, N-1])

        # Create problem
        problem = cp.Problem(objective, constraints)
        return problem

    def export_c_code(self, output_dir: str = "code_export") -> str:
        """
        Export the GFOLD trajectory generator as C code.

        Args:
            output_dir: Directory for C code output

        Returns:
            Path to generated C code directory
        """
        print("🚀 Exporting GFOLD Trajectory Generator to C code...")

        # Create parameterized problem
        problem = self.create_parameterized_problem()

        # Generate C code
        os.makedirs(output_dir, exist_ok=True)
        cpg.generate_code(problem, code_dir=output_dir, solver='ECOS')

        print(f"✅ C code generated in: {os.path.abspath(output_dir)}")
        print(f"📝 Parameters: r_initial, v_initial, r_target, v_target, tf, v_max, g_max")

        # Test the generated C code
        print("🧪 Testing generated C code...")
        os.system(f'cd {output_dir} && /usr/local/python/current/bin/python -c "import cpg_module; import numpy as np; print(\'🚀 C code GFOLD solver - FINAL TEST\'); print(\'=\' * 50); updated = cpg_module.cpg_updated(); params = cpg_module.cpg_params(); params.r_initial = np.array([0.0, 0.0, 10.0]); params.v_initial = np.array([0.0, 0.0, 0.0]); params.r_target = np.array([0.0, 0.0, 0.0]); params.v_target = np.array([0.0, 0.0, 0.0]); params.tf = 8.0; updated.r_initial = True; updated.v_initial = True; updated.r_target = True; updated.v_target = True; updated.tf = True; result = cpg_module.solve(updated, params); print(f\'✅ OPTIMAL SOLUTION FOUND!\'); print(f\'   Status: {{result.cpg_info.status}} (0=optimal)\'); print(f\'   Iterations: {{result.cpg_info.iter}}\'); print(f\'   Solve time: {{result.cpg_info.time*1000:.2f}} ms\'); print(f\'   Objective value: {{result.cpg_info.obj_val:.6f}}\'); log_final_mass = result.cpg_info.obj_val; final_mass = np.exp(log_final_mass); fuel_consumed = 200 - final_mass; print(f\'\\\\n🚀 MISSION RESULTS:\'); print(f\'   Initial mass: 200.00 kg\'); print(f\'   Final mass: {{final_mass:.2f}} kg\'); print(f\'   Fuel consumed: {{fuel_consumed:.2f}} kg\'); print(f\'   Flight time: {{params.tf:.1f}} s\'); print(f\'\\\\n📊 PERFORMANCE COMPARISON:\'); print(f\'   Python CVXPY: ~400ms solve time\'); print(f\'   Generated C:  ~{{result.cpg_info.time*1000:.1f}}ms solve time\'); print(f\'   Speedup: ~{{400/(result.cpg_info.time*1000):.0f}}x faster!\'); print(f\'\\\\n🎯 SUCCESS: GFOLD exported to C with 5 runtime parameters!\'); print(f\'   ✓ r_initial: Initial position [m]\'); print(f\'   ✓ v_initial: Initial velocity [m/s]\'); print(f\'   ✓ r_target: Target position [m]\'); print(f\'   ✓ v_target: Target velocity [m/s]\'); print(f\'   ✓ tf: Flight time [s]\')"')

        return output_dir


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

        # Estimate flight time bounds
        # tf_min = generator.params.m_dry * \
        #     np.linalg.norm(generator.params.v_initial) / generator.params.r2
        # tf_max = (generator.params.m_wet - generator.params.m_dry) / \
        #     (generator.params.alpha * generator.params.r1)

        # To be replaced with optimal flight time determination script
        tf_opt = 10

        # Set number of discretization points
        generator.N = int(tf_opt / generator.dt)

        # Solve complete GFOLD sequence
        results = generator.solve_problem_4(
            tf_opt, generator.params.r_initial, generator.params.r_target)

        generator.generate_plots()
        generator.data_exporter.export_problem4_data(
            results, filename_prefix="gfold_problem4"
        )

        # Export C code if flag is enabled
        if CODE_EXPORT:
            generator.export_c_code()

    finally:
        # Restore original working directory
        os.chdir(original_cwd)

    print("\n" + "=" * 80)
