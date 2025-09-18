# GFOLD_static_p3p4

from cvxpy import *
from time import time
import numpy as np  # *** MODIFIED: standardized alias from 'npp' to 'np'
import pandas as pd
from scipy.interpolate import interp1d
from GFOLD_Static_Parms_py3 import *
from EvilPlotting_py3 import *

''' As defined in the paper...

 PROBLEM 3: Minimum Landing Error (tf roughly solved)
 MINIMIZE : norm of landing error vector
 SUBJ TO  :
            0) initial conditions satisfied (position, velocity)
            1) final conditions satisfied (altitude, velocity)
            2) dynamics always satisfied
            3) x stays in cone at all times
            4) relaxed convexified mass and thrust constraints
            5) thrust pointing constraint
            6) sub-surface flight constraint

 PROBLEM 4: Minimum Fuel Use
 MAXIMIZE : landing mass, opt variables are dynamical and
 SUBJ TO  :
            0) same constraints as p1, plus:
            1) landing point must be equal or better than that found by p1

'''


def GFOLD_py3(inputs):  # PRIMARY GFOLD SOLVER

    if inputs[-1] == 'p3':
        program = 3
        tf_, r0, prog_flag = inputs
    elif inputs[-1] == 'p4':
        program = 4
        tf_, r0, rf_, prog_flag = inputs

    N = 120  # Fixed N, can be parameterized if needed
    dt = 0.1  # Integration dt

    print('N = ', N)
    print('dt= ', dt)
    print('r0 =', r0)
    print('v0 =', v0)

    # *** MODIFIED: Use numpy array properly, no need for Parameter()
    x0 = np.array([r0[0], r0[1], r0[2], v0[0], v0[1], v0[2]])

    # *** MODIFIED: cvxpy Variable shape must be a tuple, not separate args
    x = Variable((6, N))  # state vector (3position,3velocity)
    u = Variable((3, N))  # control thrust vector normalized by mass
    z = Variable((1, N))  # logarithm of mass
    s = Variable((1, N))  # thrust slack parameter

    con = []  # constraints list

    # *** MODIFIED: cvxpy slicing uses comma, corrected to proper slicing
    con += [x[0:3, 0] == x0[0:3]]
    con += [x[3:6, 0] == x0[3:6]]
    con += [x[3:6, N-1] == vf]  # final velocity condition

    con += [s[0, N-1] == 0]  # thrust at end must be zero, corrected indexing
    con += [u[:, 0] == s[0, 0] * np.array([1, 0, 0])]  # thrust starts straight
    # thrust ends straight
    con += [u[:, N-1] == s[0, N-1] * np.array([1, 0, 0])]
    con += [z[0, 0] == log(m_wet)]  # initial mass log value

    if program == 3:
        con += [x[0, N-1] == 0]  # final altitude constraint

    elif program == 4:
        con += [x[0:3, N-1] == rf_]  # final position constraint from problem 3

    for n in range(N-1):
        # Leapfrog Integration Method constraints
        con += [x[3:6, n+1] == x[3:6, n] +
                (dt / 2) * ((u[:, n] + g) + (u[:, n+1] + g))]
        con += [x[0:3, n+1] == x[0:3, n] +
                (dt / 2) * (x[3:6, n+1] + x[3:6, n])]

        # Glideslope constraint
        con += [norm((x[0:3, n] - rf)[0:2]) - c.T[0] * (x[0, n] - rf[0]) <= 0]

        # Velocity limit
        con += [norm(x[3:6, n]) <= V_max]

        # Mass decrease relation
        con += [z[0, n+1] == z[0, n] -
                (alpha * dt / 2) * (s[0, n] + s[0, n+1])]

        # Thrust magnitude limit
        con += [norm(u[:, n]) <= s[0, n]]

        # Thrust pointing constraint
        con += [u[0, n] >= np.cos(p_cs) * s[0, n]]

        if n > 0:
            z0_term = m_wet - alpha * r2 * n * dt
            z1_term = m_wet - alpha * r1 * n * dt
            z0 = log(z0_term)
            z1 = log(z1_term)
            mu_1 = r1 / z1_term
            mu_2 = r2 / z0_term

            # Thrust bounds and physical constraints
            con += [s[0, n] <= mu_2 * (1 - (z[0, n] - z0))]
            con += [z[0, n] >= z0]
            con += [z[0, n] <= z1]

    # Constraint: x position non-negative for all steps except last
    con += [x[0, 0:N-1] >= 0]

   # print(con)

    if program == 3:
        print('-----------------------------')
        objective = Minimize(norm(x[0:3, N-1] - rf))
        problem = Problem(objective, con)
        obj_opt = problem.solve(solver=ECOS, verbose=True, feastol=5e-20)
        print('-----------------------------')

    elif program == 4:
        print('-----------------------------')
        objective = Maximize(z[0, N-1])
        problem = Problem(objective, con)
        obj_opt = problem.solve(solver=ECOS, verbose=True)
        print('-----------------------------')

    if program == 3:
        if z.value is not None:
            # *** MODIFIED: z.value is 2D array, access first row, convert map to list for Python 3
            m = list(map(np.exp, z.value[0].tolist()))
            return obj_opt, x, u, m, (N / dt), s, z
        else:
            return obj_opt, None, None, None, (N / dt), None, None
    elif program == 4:
        if z.value is not None:
            m = list(map(np.exp, z.value[0].tolist()))
            return obj_opt, x, u, m, (N / dt), s, z
        else:
            return obj_opt, None, None, None, None, (N / dt), None, None


def P3_P4_py3(r0=r_):

    print('------------------------------------------------------------------------------------------------')
    print('                                      \033[1m GFOLD Static Solver \033[0m                   ')
    print('------------------------------------------------------------------------------------------------')

    start_ = time()

    tf_min = (m_dry * np.linalg.norm(v0 / r2))
    tf_max = (m_wet - m_dry) / (alpha * r1)
    print('min tf :%f -- max tf: %f' % (tf_min, tf_max))

    t = [60, 60]

    obj, x, u, m, tf, s, z = GFOLD_py3((t, r0, 'p3'))  # EXECUTE PROBLEM 3
    print('p3 object :%f after %f sec' % (obj, time() - start_))
    print('tf : %f' % (tf))
    print('rf :')

    if x is None:
        print('      ' + str(None))
        return None

    for r in x[0:3, -1].value:
        print('      ' + str(r))
    print()
    
    obj, x, u, m, tf, s, z = GFOLD_py3((tf, r0, rf, 'p4'))  # EXECUTE PROBLEM 4
    print('p4 object :%f after %f sec' % (obj, time() - start_))
    print('tf : %f' % (tf))
    print('rf :')
    for r in x[0:3, -1].value:
        print('      ' + str(r))
    print()

    # print('gfold took: %f sec' % (time() - start_))

    # # Debugging stuff:

    # # obj,r,v,u,m = yielded[[vector[0] for vector in yielded].index(min(tf_yield))]
    # # tf_opt = tf_array[[vector[0] for vector in yielded].index(min(tf_yield))]

    # # for var in (tf,r,v,u,m):
    # #    print('var =',var)
    # #    print(' ')
    # #    print('varval =',var.value)

    x = x.value
    u = u.value
    s = s.value
    z = z.value

    plot_run3D(tf, x, u, m, s, z)

    # Convert to numpy arrays for plotting
    r = np.array(x[0:3,:])
    v = np.array(x[3:6,:])
    z = np.array(z)
    s = np.array(s)
    u = np.array(u)
    t = np.linspace(1,tf,num=r.shape[1])
    if t.shape==() or r.shape==() or v.shape==() or u.shape==():
        print('data actually empty')
        return
    
    # Convert to 1D arrays for plotting
    x = r[1,:]
    y = r[2,:]
    z = r[0,:]
    thrust_x= [np.linalg.norm(u[:,i])* 0 for i in range(len(v.T))]
    thrust_y= [np.linalg.norm(u[:,i])* 0 for i in range(len(v.T))]
    thrust_z= [np.linalg.norm(u[:,i])* m[0] for i in range(len(v.T))]
    #vnorm = [np.linalg.norm(vel) for vel in v.T]
    if len(m) == 1:
        m = m * r.shape[1]

    # Original data
    trajectory = {
        'time': t,
        'pos_x': r[1, :],
        'pos_y': r[2, :],
        'pos_z': r[0, :],
        'vel_x': v[1, :],
        'vel_y': v[2, :],
        'vel_z': v[0, :]
    }

    # Create DataFrame
    cols = ['time', 'pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z']
    df = pd.DataFrame(trajectory, columns=cols)

    # Create new time array with 0.01s intervals
    t_new = np.arange(df['time'].iloc[0], df['time'].iloc[-1], 0.01)

    # Interpolate each column except 'time'
    interpolated_data = {'time': t_new}
    for col in cols[1:]:  # skip 'time'
        f = interp1d(df['time'], df[col], kind='linear')
        interpolated_data[col] = f(t_new)

    # Create new interpolated DataFrame
    df_interp = pd.DataFrame(interpolated_data, columns=cols)

    # Create subsets correctly
    df_interp_pos_x = pd.DataFrame({
        'pos_y': df_interp['pos_y'],
        'vel_y': df_interp['vel_y']
    })

    df_interp_height = pd.DataFrame({
        'pos_z': df_interp['pos_z'],
        'vel_z': df_interp['vel_z']
    })

    # Save to CSV
    df_interp.to_csv('database/trajectories/trajectory.csv', index=False)
    df_interp_pos_x.to_csv('database/trajectories/trajectory_posx.csv', index=False)
    df_interp_height.to_csv('database/trajectories/trajectory_height.csv', index=False)

    # Plotting - Regular versions
    trajectory_plots_planes(t,x,y,z,thrust_x,thrust_y,thrust_z, show_arrows=False)
    trajectory_plot_3d(t, x, y, z, thrust_x, thrust_y, thrust_z, show_arrows=False, earth_surface=True)
    
    # Plotting - Dark mode versions
    trajectory_plots_planes_dark(t,x,y,z,thrust_x,thrust_y,thrust_z, show_arrows=False)
    trajectory_plot_3d_dark(t, x, y, z, thrust_x, thrust_y, thrust_z, show_arrows=False, earth_surface=True)

    return obj, x, u, m, tf


if __name__ == '__main__':
    P3_P4_py3(r_)
