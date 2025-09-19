"""
GFOLD Trajectory Plotting Module

This module provides comprehensive plotting functionality for GFOLD trajectory visualization.
Includes both regular and dark theme variants for 3D trajectories and planar projections.

Based on ARCHIVED_EvilPlotting_py3.py with        # Add shared colorbar with medium height
        cax = fig.add_axes([0.92, 0.5, 0.02, 0.35])
        sm = cm.ScalarMappable(norm=norm, cmap=cmap)
        sm.set_array(t)
        cbar = fig.colorbar(sm, cax=cax)
        cbar.set_label("Time (s)")nizations for integration with trajectory_generator.py
"""

from typing import Tuple, Optional
import os
import matplotlib.gridspec as gridspec
from matplotlib.patches import FancyArrowPatch
from matplotlib.colors import Normalize
from matplotlib.collections import LineCollection
import matplotlib.cm as cm
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.use('Agg')  # Set non-GUI backend for server environments


class TrajectoryPlotter:
    """
    Comprehensive plotting class for GFOLD trajectory visualization.

    Provides methods for creating 3D trajectory plots and planar projections
    with support for both light and dark themes.
    """

    def __init__(self, output_dir: str = "results/images"):
        """
        Initialize the trajectory plotter.

        Args:
            output_dir: Directory to save plot images
        """
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

    def _prepare_trajectory_data(self, x: np.ndarray, u: np.ndarray, m: np.ndarray,
                                 tf: float) -> Tuple[np.ndarray, ...]:
        """
        Prepare trajectory data for plotting.

        Args:
            x: State trajectory [6 x N]
            u: Control trajectory [3 x N] 
            m: Mass trajectory [N]
            tf: Flight time

        Returns:
            Tuple of (time, positions, velocities, thrust_vectors)
        """
        N = x.shape[1]
        t = np.linspace(0, tf, N)

        # Extract positions and velocities
        pos = x[0:3, :]  # [x, y, z] coordinates
        vel = x[3:6, :]

        # Convert positions to plotting coordinates
        # New z-up system: [x, y, z] where z is height
        # Plot coordinates: [x, y, z] for 3D visualization
        x_pos = pos[0, :]  # x-coordinate (east-west)
        y_pos = pos[1, :]  # y-coordinate (north-south)
        z_pos = pos[2, :]  # z-coordinate (altitude/height)

        # Compute thrust vectors (normalized by mass for visualization)
        if len(m) == 1:
            m = np.full(N, m[0])

        # No x-component for vis
        thrust_x = np.array([np.linalg.norm(u[:, i]) * 0 for i in range(N)])
        # No y-component for vis
        thrust_y = np.array([np.linalg.norm(u[:, i]) * 0 for i in range(N)])
        thrust_z = np.array([np.linalg.norm(u[:, i]) * m[i]
                            for i in range(N)])  # Z-thrust visualization

        return t, x_pos, y_pos, z_pos, thrust_x, thrust_y, thrust_z

    def plot_3d_trajectory(self, x: np.ndarray, u: np.ndarray, m: np.ndarray,
                           tf: float, dark_theme: bool = False,
                           filename: Optional[str] = None) -> str:
        """
        Create 3D trajectory plot.

        Args:
            x: State trajectory [6 x N]
            u: Control trajectory [3 x N]
            m: Mass trajectory [N] 
            tf: Flight time
            dark_theme: Use dark theme if True
            filename: Custom filename (auto-generated if None)

        Returns:
            Path to saved plot file
        """
        t, x_pos, y_pos, z_pos, thrust_x, thrust_y, thrust_z = self._prepare_trajectory_data(
            x, u, m, tf)

        if dark_theme:
            plt.style.use('dark_background')
            theme_suffix = "_dark"
        else:
            plt.style.use('default')
            theme_suffix = ""

        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')

        # Color trajectory by time
        norm = Normalize(vmin=t.min(), vmax=t.max())
        cmap = cm.plasma if dark_theme else cm.viridis

        # Plot trajectory with time-based coloring
        for i in range(len(t) - 1):
            ax.plot3D([x_pos[i], x_pos[i+1]],
                      [y_pos[i], y_pos[i+1]],
                      [z_pos[i], z_pos[i+1]],
                      color=cmap(norm(t[i])), linewidth=2)

        # Add start and end markers (more subtle)
        start_color = 'lime' if dark_theme else 'green'
        end_color = 'red'

        ax.scatter([x_pos[0]], [y_pos[0]], [z_pos[0]],
                   color=start_color, s=60, alpha=0.7, label='Start', marker='o',
                   edgecolors='white', linewidth=0.5)
        ax.scatter([x_pos[-1]], [y_pos[-1]], [z_pos[-1]],
                   color=end_color, s=60, alpha=0.7, label='Landing', marker='X',
                   edgecolors='white', linewidth=0.5)

        # Add surface plane (ground level)
        if z_pos.min() <= 0:
            xx, yy = np.meshgrid(np.linspace(x_pos.min()-5, x_pos.max()+5, 10),
                                 np.linspace(y_pos.min()-5, y_pos.max()+5, 10))
            zz = np.zeros_like(xx)
            surface_color = 'gray' if dark_theme else 'lightgray'
            ax.plot_surface(xx, yy, zz, alpha=0.3, color=surface_color)

        # Labels and title
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Altitude (m)')

        title = f'GFOLD 3D Trajectory (Flight Time: {tf:.1f}s)'
        ax.set_title(title)

        ax.legend()

        # Add colorbar for time
        sm = cm.ScalarMappable(norm=norm, cmap=cmap)
        sm.set_array(t)
        cbar = fig.colorbar(sm, ax=ax, shrink=0.5)
        cbar.set_label("Time (s)")

        # Save plot
        if filename is None:
            filename = f"gfold_3d_trajectory{theme_suffix}.png"

        filepath = os.path.join(self.output_dir, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight',
                    facecolor='black' if dark_theme else 'white')
        plt.close()

        # print(f"Saved 3D trajectory plot to {filepath}")
        return filepath

    def plot_trajectory_planes(self, x: np.ndarray, u: np.ndarray, m: np.ndarray,
                               tf: float, dark_theme: bool = False,
                               filename: Optional[str] = None,
                               gravity: Optional[np.ndarray] = None) -> str:
        """
        Create trajectory projections on XY, XZ, and YZ planes.

        Args:
            x: State trajectory [6 x N]
            u: Control trajectory [3 x N]
            m: Mass trajectory [N]
            tf: Flight time  
            dark_theme: Use dark theme if True
            filename: Custom filename (auto-generated if None)

        Returns:
            Path to saved plot file
        """
        t, x_pos, y_pos, z_pos, thrust_x, thrust_y, thrust_z = self._prepare_trajectory_data(
            x, u, m, tf)

        if dark_theme:
            plt.style.use('dark_background')
            theme_suffix = "_dark"
            arrow_color = 'cyan'
        else:
            plt.style.use('default')
            theme_suffix = ""
            arrow_color = 'blue'

        # Create figure with subplots
        fig = plt.figure(figsize=(15, 12))
        gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.3, wspace=0.3)

        # Color mapping for time
        norm = Normalize(vmin=t.min(), vmax=t.max())
        cmap = cm.plasma if dark_theme else cm.viridis

        def plot_trajectory_projection(ax, coord1, coord2, label1, label2, title):
            """Helper function to plot trajectory projection."""
            # Plot trajectory with time-based coloring
            points = np.array([coord1, coord2]).T.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            lc = LineCollection(segments, cmap=cmap, norm=norm)
            lc.set_array(t)
            lc.set_linewidth(2)
            ax.add_collection(lc)

            # Add start and end markers (more subtle)
            start_color = 'lime' if dark_theme else 'green'
            end_color = 'red'
            ax.scatter(coord1[0], coord2[0], color=start_color, s=40, alpha=0.7,
                       label='Start', marker='o', zorder=5, edgecolors='white', linewidth=0.5)
            ax.scatter(coord1[-1], coord2[-1], color=end_color, s=40, alpha=0.7,
                       label='Landing', marker='X', zorder=5, edgecolors='white', linewidth=0.5)

            ax.set_xlabel(label1)
            ax.set_ylabel(label2)
            ax.set_title(title)
            ax.grid(True, alpha=0.3)
            ax.legend()
            # Use consistent aspect ratio for all subplots instead of equal
            ax.set_aspect('auto')

        # XY plane (X vs Y horizontal projection)
        ax1 = fig.add_subplot(gs[0, 0])
        plot_trajectory_projection(ax1, x_pos, y_pos,
                                   'X Position (m)', 'Y Position (m)',
                                   'XY Plane (Top View)')

        # XZ plane (X vs Altitude)
        ax2 = fig.add_subplot(gs[0, 1])
        plot_trajectory_projection(ax2, x_pos, z_pos,
                                   'X Position (m)', 'Altitude (m)',
                                   'XZ Plane (Side View)')

        # YZ plane (Y vs Altitude)
        ax3 = fig.add_subplot(gs[0, 2])
        plot_trajectory_projection(ax3, y_pos, z_pos,
                                   'Y Position (m)', 'Altitude (m)',
                                   'YZ Plane (Front View)')

        # Combined velocity and acceleration components plot
        ax4 = fig.add_subplot(gs[1, :])

        # Calculate velocity magnitude
        vel_mag = np.sqrt(x[3, :]**2 + x[4, :]**2 + x[5, :]**2)

        # Plot velocity magnitude on primary y-axis
        line1 = ax4.plot(t, vel_mag, linewidth=2,
                         color='orange' if dark_theme else 'blue',
                         label='Velocity Magnitude')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Velocity Magnitude (m/s)',
                       color='orange' if dark_theme else 'blue')
        ax4.tick_params(
            axis='y', labelcolor='orange' if dark_theme else 'blue')

        # Create secondary y-axis for acceleration components
        ax4_acc = ax4.twinx()

        if u is not None:
            # Calculate total acceleration components (thrust + gravity)
            g_vec = gravity if gravity is not None else np.array(
                [0, 0, -9.80665])  # Default gravity vector
            # Add gravity to thrust acceleration
            acc_total = u + g_vec.reshape(-1, 1)

            # Plot directional acceleration components
            acc_colors = {
                'x': 'red' if dark_theme else 'darkred',
                'y': 'green' if dark_theme else 'darkgreen',
                'z': 'blue' if dark_theme else 'darkblue'
            }

            line2_x = ax4_acc.plot(t, acc_total[0, :], linewidth=2,
                                   color=acc_colors['x'], linestyle='-',
                                   label='Acc X', alpha=0.8)
            line2_y = ax4_acc.plot(t, acc_total[1, :], linewidth=2,
                                   color=acc_colors['y'], linestyle='--',
                                   label='Acc Y', alpha=0.8)
            line2_z = ax4_acc.plot(t, acc_total[2, :], linewidth=2,
                                   color=acc_colors['z'], linestyle='-.',
                                   label='Acc Z', alpha=0.8)

            line2 = line2_x + line2_y + line2_z
            ax4_acc.set_ylabel('Acceleration Components (m/s²)',
                               color='black' if not dark_theme else 'white')
            ax4_acc.tick_params(
                axis='y', labelcolor='black' if not dark_theme else 'white')

            # Add horizontal line at zero acceleration for reference
            ax4_acc.axhline(y=0, color='gray', linestyle=':',
                            alpha=0.5, linewidth=1)
        else:
            # If no control data, show zero acceleration
            line2 = ax4_acc.plot(t, np.zeros_like(t), linewidth=2, color='gray',
                                 label='Acceleration (No Data)')
            ax4_acc.set_ylabel('Acceleration Components (m/s²)', color='gray')
            ax4_acc.tick_params(axis='y', labelcolor='gray')

        # Add legend for both lines
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax4.legend(lines, labels, loc='upper left')

        ax4.set_title('Velocity Magnitude and Acceleration Components')
        ax4.grid(True, alpha=0.3)

        # Add overall title
        title = f'GFOLD Trajectory Analysis (Flight Time: {tf:.1f}s)'
        fig.suptitle(title, fontsize=16)

        # Add shared colorbar positioned for the top row of subplots
        cax = fig.add_axes([0.92, 0.55, 0.02, 0.329])
        sm = cm.ScalarMappable(norm=norm, cmap=cmap)
        sm.set_array(t)
        cbar = fig.colorbar(sm, cax=cax)
        cbar.set_label("Time (s)")

        # Save plot
        if filename is None:
            filename = f"gfold_trajectory_planes{theme_suffix}.png"

        filepath = os.path.join(self.output_dir, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight',
                    facecolor='black' if dark_theme else 'white')
        plt.close()

        # print(f"Saved trajectory planes plot to {filepath}")
        return filepath

    def plot_all_trajectories(self, x: np.ndarray, u: np.ndarray, m: np.ndarray,
                              tf: float, gravity: Optional[np.ndarray] = None) -> list:
        """
        Generate all trajectory plots (3D and planes, both themes).

        Args:
            x: State trajectory [6 x N]
            u: Control trajectory [3 x N] 
            m: Mass trajectory [N]
            tf: Flight time

        Returns:
            List of saved file paths
        """
        saved_files = []

        # Generate all plot variants
        saved_files.append(self.plot_3d_trajectory(
            x, u, m, tf, dark_theme=False))
        saved_files.append(self.plot_3d_trajectory(
            x, u, m, tf, dark_theme=True))
        saved_files.append(self.plot_trajectory_planes(
            x, u, m, tf, dark_theme=False, gravity=gravity))
        saved_files.append(self.plot_trajectory_planes(
            x, u, m, tf, dark_theme=True, gravity=gravity))

        # print(f"\nGenerated {len(saved_files)} trajectory plots:")
        # for filepath in saved_files:
        #     print(f"  - {filepath}")
        print("\n✓ Problem 4 plot export completed successfully!")

        return saved_files


def plot_gfold_results(x: np.ndarray, u: np.ndarray, m: np.ndarray, tf: float,
                       output_dir: str = "results/images") -> list:
    """
    Convenience function to generate all GFOLD trajectory plots.

    Args:
        x: State trajectory [6 x N]
        u: Control trajectory [3 x N]
        m: Mass trajectory [N] 
        tf: Flight time
        output_dir: Directory to save plots

    Returns:
        List of saved file paths
    """
    plotter = TrajectoryPlotter(output_dir)
    return plotter.plot_all_trajectories(x, u, m, tf)
