import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.use('Agg')  # Set non-GUI backend before importing pyplot

from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import GFOLD_Static_Parms_py3 as p
import matplotlib.cm as cm
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib.patches import FancyArrowPatch
import matplotlib.gridspec as gridspec


def plot_run2D_py3(t, r, v, u, m):
    '''
    print('r =',r)
    print('v =',v)
    print('u =',u)
    print('m =',m)
    print('s =',s)
    '''
    # assume inputs are already NumPy arrays (or lists convertible to arrays)
    r = np.array(r)
    v = np.array(v)
    u = np.array(u)
    T_val = [np.linalg.norm(u[:, i])*m[i] for i in range(len(v.T))]
    vnorm = [np.linalg.norm(vel) for vel in v.T]

    # u_dirs = 90 - np.degrees(np.atan2(u[1,:], u[0,:]))
    # T_vals = np.multiply(u_norms , m)

    traj = plt.figure()

    # plt.subplot(4,1,1)
    plt.plot(r[0, :], r[1, :])
    M = str(np.tan(np.radians(p.slope)))
    nM = str(-float(M))
    bx = str(p.r_d[0])
    by = str(p.r_d[1])
    x = np.array(range(0, int(max(r[0, :]))))
    plt.plot(x, eval(M+'*(x-'+bx+')+'+by))
    x = np.array(range(int(min(r[0, :])), 0))
    plt.plot(x, eval(nM+'*(x-'+bx+')+'+by))
    plt.title('Position (m)')

    f = plt.figure()
    ax = f.add_subplot(411)

    plt.plot(t, vnorm)
    by = str(p.V_max)
    x = np.array(range(0, int(max(t))))
    plt.plot(x, eval(by))
    plt.xlabel(r"$t$", fontsize=16)
    plt.title('Velocity Magnitude (m/s)')

    plt.subplot(4, 1, 2)
    plt.plot(t, r[1, :])
    plt.xlabel(r"$t$", fontsize=16)
    plt.title('Altitude (m)')

    plt.subplot(4, 1, 3)
    plt.plot(t, m)
    plt.title('Mass (kg)')

    plt.subplot(4, 1, 4)
    plt.plot(t, T_val)
    by = str(p.T_max)
    x = np.array(range(0, int(max(t))))
    plt.plot(x, eval(by))
    plt.title('Thrust (N)')

    # plt.tight_layout()
    plt.subplots_adjust(hspace=0)
    plt.show()


def plot_run3D(tf, x, u, m, s, z):

    # print('tf',tf)

    r = np.array(x[0:3,:])
    v = np.array(x[3:6,:])
    z = np.array(z)
    s = np.array(s)
    u = np.array(u)
    t = np.linspace(1,tf,num=r.shape[1])
    if len(m) == 1:
        m = m * r.shape[1]

    # print('t',t.shape)
    # print('r',r.shape)
    # print('v',v.shape)
    # print('u',u.shape)
    # print('m',len(m))
    # print('s',s.shape)
    # print('z',z.shape)

    if t.shape==() or r.shape==() or v.shape==() or u.shape==():
        print('data actually empty')
        return

    # Th= [np.linalg.norm(u[:,i])*m[i] for i in range(len(v.T))]
    Th= [np.linalg.norm(u[:,i])* m[0] for i in range(len(v.T))]
    vnorm = [np.linalg.norm(vel) for vel in v.T]

    #u_dirs_1 = [90 - np.degrees(np.atan2(u[0,n], u[1,n])) for n in range(p.N)]
    #u_dirs_2 = [90 - np.degrees(np.atan2(u[0,n], u[2,n])) for n in range(p.N)]

    traj = plt.figure()
    ax = traj.add_subplot(111, projection='3d')
    ax.set_box_aspect([1,1,1])  # equivalent to set_aspect('equal') for 3D

    r_= np.linspace(0, max(max(r[1,:]),max(r[2,:])), 7)
    a_= np.linspace(0, 2*np.pi, 20)
    R, P = np.meshgrid(r_, a_)
    X, Y, Z = R*np.cos(P), R*np.sin(P), R*(np.tan(p.y_gs))
    #X,Y,Z=R*np.cos(P), R*np.sin(P),((R**2 - 1)**2)

    #ax.plot(x(t),y(t),z(t),label='Flight Path')
    ax.plot(r[1,:],r[2,:],r[0,:],label='Flight Path')
    ax.plot_surface(X, Y, Z, cmap=plt.cm.YlGnBu_r)

    # Tweak the limits and add latex math labels.

    ax.set_xlabel(r'$x{1}$')
    ax.set_ylabel(r'$x{2}$')
    ax.set_zlabel(r'$x{0}$')

    ax.legend()

    f = plt.figure()
    ax = f.add_subplot(511)

    plt.plot(t,vnorm)
    y=str(p.V_max)
    x=np.array(range(0,int(max(t))))
    plt.plot(x,eval('0*x+'+y))
    plt.title('Velocity Magnitude (m/s)')

    plt.subplot(5,1,2)
    plt.plot(t,r[0,:])
    plt.title('Altitude (m)')

    plt.subplot(5,1,3)
    plt.plot(t,m) # m*len(t)
    plt.title('Mass (kg)')

    plt.subplot(5,1,4)
    plt.plot(t,Th)
    y=str(p.T_max)
    x=np.array(range(0,int(max(t))))
    plt.plot(x,eval('0*x+'+y))
    plt.title('Thrust (N)')

    z0_term = (p.m_wet - p.alpha * p.r2)  # see ref [2], eq 34,35,36
    z1_term = (p.m_wet - p.alpha * p.r1)
    lim=[]
    lim2=[]
    n=0
    z=z.flatten()
    for t_ in t:
        if t_ > 0:
            try:
                v = p.r2/(z0_term*t_) * (1 - (z[n] - np.log(z0_term*t_)))
            except ZeroDivisionError:
                v = 0
            lim.append( v )
            try:
                v = p.r1/(z1_term*t_) *(1 - (z[n] - np.log(z0_term*t_)) + (1/2)*(z[n] - np.log(z0_term*t_))**2 )
            except ZeroDivisionError:
                v = 0
            lim2.append( v )
        else:
            lim.append(0)
            lim2.append(0)
        n+=1
    lim = np.array(lim).flatten()
    plt.subplot(5,1,5)
    plt.plot(t,lim)
    plt.plot(t,lim2)
    s = s.flatten()
    if s.shape == (1,65):
        s.reshape((65,))
        print('reshape',s)
        print('s',s)
    plt.plot(t,s)
    plt.title('Sigma Slack')

    #plt.tight_layout()
    plt.subplots_adjust(hspace=0)

    # Save the plot only
    plt.savefig("img/gfold_3d.png", dpi=300)
    print("Saved 3D GFOLD plot to gfold_3d.png")


def trajectory_plot_3d(t, x, y, z, thrust_x, thrust_y, thrust_z, show_arrows=True, earth_surface=True):
    """
    Plots the 3D trajectory of a spacecraft with thrust arrows.
    Parameters:
    - t: Time vector
    - x: Horizontal position vector (x)
    - y: Side position vector (y)
    - z: Altitude vector (z)
    - thrust_x: Thrust vector in the x direction
    - thrust_y: Thrust vector in the y direction
    - thrust_z: Thrust vector in the z direction
    - show_arrows: Boolean to toggle thrust arrows
    - earth_surface: Boolean to toggle Earth surface plane
    """

    # Normalize time for colormap
    norm = Normalize(vmin=t[0], vmax=t[-1])
    cmap = cm.get_cmap('jet')
    colors = cmap(norm(t))

    # Create figure and 3D axis
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("3D Trajectory with Time of Flight")

    # Plot main trajectory
    for i in range(len(t) - 1):
        ax.plot(x[i:i+2], y[i:i+2], z[i:i+2], color=colors[i], linewidth=2)

    # Arrow drawing: use fixed max length for 100% thrust
    if show_arrows:
        step = max(1, len(t) // 50)
        thrust_vectors = np.vstack([thrust_x, thrust_y, thrust_z])
        thrust_mags = np.linalg.norm(thrust_vectors, axis=0)
        max_thrust = thrust_mags.max()

        # Set 100% thrust length (absolute units)
        fixed_arrow_length = 2.0  # change this if you want longer/shorter arrows

        for i in range(0, len(t), step):
            mag = thrust_mags[i]
            if mag == 0:
                continue

            # Normalized direction
            direction = thrust_vectors[:, i] / mag
            length = (mag / max_thrust) * fixed_arrow_length

            ax.plot(
                [x[i], x[i] + direction[0] * length],
                [y[i], y[i] + direction[1] * length],
                [z[i], z[i] + direction[2] * length],
                color='blue', linewidth=1.5
            )

    # Labels
    ax.set_xlabel("X (horizontal)")
    ax.set_ylabel("Y (lateral)")
    ax.set_zlabel("Z (altitude)")

    # Colorbar for time
    sm = cm.ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array(t)
    cbar = fig.colorbar(sm, ax=ax, shrink=0.7, pad=0.02, fraction=0.05)
    cbar.set_label("Time (s)")

    # Optional: Better zoom framing
    margin = 2
    ax.set_xlim(x.min() - margin, x.max() + margin)
    ax.set_ylim(y.min() - margin, y.max() + margin)

    # Set z-limits with ground at 0 if earth_surface is enabled
    if earth_surface:
        ax.set_zlim(0, z.max() + margin)
        
        # Create grid for Earth surface at z=0
        xx, yy = np.meshgrid(
            np.linspace(x.min() - margin, x.max() + margin, 20),
            np.linspace(y.min() - margin, y.max() + margin, 20)
        )
        zz = np.zeros_like(xx)
        
        ax.plot_surface(xx, yy, zz, color='grey', alpha=0.4)

    else:
        ax.set_zlim(z.min() - margin, z.max() + margin)

    # Optional: Adjust camera angle
    ax.view_init(elev=10, azim=105)

    #plt.tight_layout()
    # Save the plot only
    plt.savefig("img/gfold_3d_trajectory.png", dpi=300)
    print("Saved 3D GFOLD trajectory plot to gfold_3d_trajectory.png")


def trajectory_plots_planes(t,x,y,z,thrust_x,thrust_y,thrust_z, show_arrows=True):
    """
    Plots the trajectory of a spacecraft in three different planes:
    1. XY plane (x vs z)
    2. ZY plane (y vs z)
    3. Ground track (x vs y)
    Each plot includes arrows indicating the thrust direction and magnitude.
    Parameters:
    - t: Time vector
    - x: Horizontal position vector (x)
    - y: Side position vector (y)
    - z: Altitude vector (z)
    - thrust_x: Thrust vector in the x direction
    - thrust_y: Thrust vector in the y direction
    - thrust_z: Thrust vector in the z direction
    """

    # Time colormap normalization
    norm = Normalize(vmin=t[0], vmax=t[-1])
    cmap = cm.get_cmap('jet')

    # GridSpec to align subplots and colorbar
    fig = plt.figure(figsize=(14, 16))
    gs = gridspec.GridSpec(3, 2, width_ratios=[20, 1], height_ratios=[1, 1, 1], wspace=0.05, hspace=0.15)

    ax1 = fig.add_subplot(gs[0, 0])  # Ground track: x vs y
    ax2 = fig.add_subplot(gs[1, 0])  # XY plane: x vs z
    ax3 = fig.add_subplot(gs[2, 0])  # ZY plane: y vs z
    cax = fig.add_subplot(gs[:, 1])  # Colorbar spanning all rows

    # Plotting function
    def plot_trajectory_with_arrows(ax, coord1, coord2, thrust1, thrust2, label1, label2, arrow_color='blue', show_arrows=show_arrows):
        """
        Plots the trajectory in a specified plane with arrows indicating thrust.
        Parameters:
        - ax: Matplotlib axis to plot on
        - coord1: First coordinate vector (e.g., x or y)
        - coord2: Second coordinate vector (e.g., z or y)
        - thrust1: Thrust vector in the direction of coord1
        - thrust2: Thrust vector in the direction of coord2
        - label1: Label for the first coordinate
        - label2: Label for the second coordinate
        - arrow_color: Color of the arrows indicating thrust
        """
        points = np.array([coord1, coord2]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segments, cmap=cmap, norm=norm)
        lc.set_array(t)
        lc.set_linewidth(4)
        ax.add_collection(lc)

        ax.set_xlim(coord1.min() - 1, coord1.max() + 1)
        ax.set_ylim(coord2.min() - 1, coord2.max() + 1)
        ax.set_xlabel(label1)
        ax.set_ylabel(label2)
        ax.grid(True)

        thrust_mags = np.linalg.norm(np.vstack([thrust1, thrust2]), axis=0)
        max_thrust = thrust_mags.max()

        x0, x1 = ax.get_xlim()
        y0, y1 = ax.get_ylim()
        max_arrow_len_axes = 0.05

        if show_arrows:

            step = max(1, len(t)//70)
            for i in range(0, len(t), step):
                direction = np.array([thrust1[i], thrust2[i]])
                mag = np.linalg.norm(direction)
                if mag == 0:
                    continue
                direction /= mag
                arrow_dx = direction[0] * (x1 - x0) * max_arrow_len_axes * (mag / max_thrust)
                arrow_dy = direction[1] * (y1 - y0) * max_arrow_len_axes * (mag / max_thrust)

                arrow = FancyArrowPatch(
                    posA=(coord1[i], coord2[i]),
                    posB=(coord1[i] + arrow_dx, coord2[i] + arrow_dy),
                    arrowstyle='-|>',
                    mutation_scale=15,
                    color=arrow_color,
                    linewidth=1.5,
                    zorder=5
                )
                ax.add_patch(arrow)


    # XY plane (x vs z)
    plot_trajectory_with_arrows(ax1, x, z, thrust_x, thrust_z, "Horizontal Position (x)", "Altitude (z)")

    # Ground track (x vs y)
    plot_trajectory_with_arrows(ax2, x, y, thrust_x, thrust_y, "Horizontal Position (x)", "Side Position (y)")

    # ZY plane (y vs z)
    plot_trajectory_with_arrows(ax3, y, z, thrust_y, thrust_z, "Side Position (y)", "Altitude (z)")

    # Shared colorbar for time
    sm = cm.ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array(t)
    cbar = fig.colorbar(sm, cax=cax)
    cbar.set_label("Time (s)")

    #plt.tight_layout()
    # Save the plot only
    plt.savefig("img/gfold_3d_trajectory_planes.png", dpi=300)
    print("Saved 3D GFOLD trajectory planes plot to gfold_3d_trajectory_planes.png")


def trajectory_plot_3d_dark(t, x, y, z, thrust_x, thrust_y, thrust_z, show_arrows=True, earth_surface=True):
    """
    Dark mode version of the 3D trajectory plot.
    """
    
    # Set dark theme
    plt.style.use('dark_background')
    
    # Normalize time for colormap
    norm = Normalize(vmin=t[0], vmax=t[-1])
    cmap = cm.get_cmap('plasma')  # Better colormap for dark background
    colors = cmap(norm(t))

    # Create figure and 3D axis
    fig = plt.figure(figsize=(14, 10), facecolor='black')
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor('black')
    ax.set_title("3D Trajectory with Time of Flight", color='white', fontsize=16)

    # Plot main trajectory
    for i in range(len(t) - 1):
        ax.plot(x[i:i+2], y[i:i+2], z[i:i+2], color=colors[i], linewidth=3)

    # Arrow drawing: use fixed max length for 100% thrust
    if show_arrows:
        step = max(1, len(t) // 50)
        thrust_vectors = np.vstack([thrust_x, thrust_y, thrust_z])
        thrust_mags = np.linalg.norm(thrust_vectors, axis=0)
        max_thrust = thrust_mags.max()

        # Set 100% thrust length (absolute units)
        fixed_arrow_length = 2.0

        for i in range(0, len(t), step):
            mag = thrust_mags[i]
            if mag == 0:
                continue

            # Normalized direction
            direction = thrust_vectors[:, i] / mag
            length = (mag / max_thrust) * fixed_arrow_length

            ax.plot(
                [x[i], x[i] + direction[0] * length],
                [y[i], y[i] + direction[1] * length],
                [z[i], z[i] + direction[2] * length],
                color='cyan', linewidth=2, alpha=0.8
            )

    # Labels with white color
    ax.set_xlabel("X (horizontal)", color='white', fontsize=12)
    ax.set_ylabel("Y (lateral)", color='white', fontsize=12)
    ax.set_zlabel("Z (altitude)", color='white', fontsize=12)
    
    # Set tick colors
    ax.tick_params(colors='white')

    # Colorbar for time
    sm = cm.ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array(t)
    cbar = fig.colorbar(sm, ax=ax, shrink=0.7, pad=0.02, fraction=0.05)
    cbar.set_label("Time (s)", color='white')
    cbar.ax.yaxis.set_tick_params(color='white')
    cbar.ax.yaxis.label.set_color('white')
    cbar.outline.set_edgecolor('white')

    # Optional: Better zoom framing
    margin = 2
    ax.set_xlim(x.min() - margin, x.max() + margin)
    ax.set_ylim(y.min() - margin, y.max() + margin)

    # Set z-limits with ground at 0 if earth_surface is enabled
    if earth_surface:
        ax.set_zlim(0, z.max() + margin)
        
        # Create grid for Earth surface at z=0 with dark styling
        xx, yy = np.meshgrid(
            np.linspace(x.min() - margin, x.max() + margin, 20),
            np.linspace(y.min() - margin, y.max() + margin, 20)
        )
        zz = np.zeros_like(xx)
        
        ax.plot_surface(xx, yy, zz, color='darkslategrey', alpha=0.6)

    else:
        ax.set_zlim(z.min() - margin, z.max() + margin)

    # Optional: Adjust camera angle
    ax.view_init(elev=10, azim=105)

    # Make panes dark
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor('white')
    ax.yaxis.pane.set_edgecolor('white')
    ax.zaxis.pane.set_edgecolor('white')
    ax.xaxis.pane.set_alpha(0.1)
    ax.yaxis.pane.set_alpha(0.1)
    ax.zaxis.pane.set_alpha(0.1)

    # Save the plot only
    plt.savefig("img/gfold_3d_trajectory_dark.png", dpi=300, facecolor='black')
    print("Saved 3D GFOLD trajectory dark plot to gfold_3d_trajectory_dark.png")
    
    # Reset style to default
    plt.style.use('default')


def trajectory_plots_planes_dark(t,x,y,z,thrust_x,thrust_y,thrust_z, show_arrows=True):
    """
    Dark mode version of the trajectory planes plot.
    """
    
    # Set dark theme
    plt.style.use('dark_background')

    # Time colormap normalization
    norm = Normalize(vmin=t[0], vmax=t[-1])
    cmap = cm.get_cmap('plasma')  # Better colormap for dark background

    # GridSpec to align subplots and colorbar
    fig = plt.figure(figsize=(14, 16), facecolor='black')
    gs = gridspec.GridSpec(3, 2, width_ratios=[20, 1], height_ratios=[1, 1, 1], wspace=0.05, hspace=0.15)

    ax1 = fig.add_subplot(gs[0, 0], facecolor='black')  # Ground track: x vs y
    ax2 = fig.add_subplot(gs[1, 0], facecolor='black')  # XY plane: x vs z
    ax3 = fig.add_subplot(gs[2, 0], facecolor='black')  # ZY plane: y vs z
    cax = fig.add_subplot(gs[:, 1])  # Colorbar spanning all rows

    # Plotting function
    def plot_trajectory_with_arrows_dark(ax, coord1, coord2, thrust1, thrust2, label1, label2, arrow_color='cyan', show_arrows=show_arrows):
        """
        Dark mode version of trajectory plotting with arrows.
        """
        points = np.array([coord1, coord2]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segments, cmap=cmap, norm=norm)
        lc.set_array(t)
        lc.set_linewidth(4)
        ax.add_collection(lc)

        ax.set_xlim(coord1.min() - 1, coord1.max() + 1)
        ax.set_ylim(coord2.min() - 1, coord2.max() + 1)
        ax.set_xlabel(label1, color='white', fontsize=12)
        ax.set_ylabel(label2, color='white', fontsize=12)
        ax.grid(True, color='grey', alpha=0.3)
        ax.tick_params(colors='white')

        thrust_mags = np.linalg.norm(np.vstack([thrust1, thrust2]), axis=0)
        max_thrust = thrust_mags.max()

        x0, x1 = ax.get_xlim()
        y0, y1 = ax.get_ylim()
        max_arrow_len_axes = 0.05

        if show_arrows:
            step = max(1, len(t)//70)
            for i in range(0, len(t), step):
                direction = np.array([thrust1[i], thrust2[i]])
                mag = np.linalg.norm(direction)
                if mag == 0:
                    continue
                direction /= mag
                arrow_dx = direction[0] * (x1 - x0) * max_arrow_len_axes * (mag / max_thrust)
                arrow_dy = direction[1] * (y1 - y0) * max_arrow_len_axes * (mag / max_thrust)

                arrow = FancyArrowPatch(
                    posA=(coord1[i], coord2[i]),
                    posB=(coord1[i] + arrow_dx, coord2[i] + arrow_dy),
                    arrowstyle='-|>',
                    mutation_scale=15,
                    color=arrow_color,
                    linewidth=1.5,
                    zorder=5,
                    alpha=0.8
                )
                ax.add_patch(arrow)

    # XY plane (x vs z)
    plot_trajectory_with_arrows_dark(ax1, x, z, thrust_x, thrust_z, "Horizontal Position (x)", "Altitude (z)")

    # Ground track (x vs y)
    plot_trajectory_with_arrows_dark(ax2, x, y, thrust_x, thrust_y, "Horizontal Position (x)", "Side Position (y)")

    # ZY plane (y vs z)
    plot_trajectory_with_arrows_dark(ax3, y, z, thrust_y, thrust_z, "Side Position (y)", "Altitude (z)")

    # Shared colorbar for time
    sm = cm.ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array(t)
    cbar = fig.colorbar(sm, cax=cax)
    cbar.set_label("Time (s)", color='white')
    cbar.ax.yaxis.set_tick_params(color='white')
    cbar.ax.yaxis.label.set_color('white')
    cbar.outline.set_edgecolor('white')

    # Save the plot only
    plt.savefig("img/gfold_3d_trajectory_planes_dark.png", dpi=300, facecolor='black')
    print("Saved 3D GFOLD trajectory planes dark plot to gfold_3d_trajectory_planes_dark.png")
    
    # Reset style to default
    plt.style.use('default')
