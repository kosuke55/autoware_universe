#!/usr/bin/env python3
"""
Bezier Curve Visualizer with Dynamic Parameter Control

This script visualizes Bezier curves with interactive controls for:
- Control points
- Velocity coefficients (v_init_coeff, v_final_coeff)
- Acceleration coefficient
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from scipy.special import comb


class BezierCurveVisualizer:
    """Interactive Bezier curve visualizer with dynamic parameter control."""

    def __init__(self):
        """Initialize the visualizer with default parameters."""
        # Default control points (4-point cubic Bezier)
        self.control_points = np.array([
            [0.0, 0.0],
            [1.0, 2.0],
            [3.0, 2.0],
            [4.0, 0.0]
        ])

        # Bezier parameters (matching bezier_pull_over.cpp)
        self.v_init_coeff = 0.5
        self.v_final_coeff = 0.1
        self.acc_coeff = 5.0

        # Number of points for curve rendering
        self.num_points = 100

        # Setup the plot
        self.setup_plot()

    def bernstein_poly(self, i, n, t):
        """
        Calculate Bernstein polynomial.

        Args:
            i: Polynomial index
            n: Polynomial degree
            t: Parameter value [0, 1]

        Returns:
            Bernstein polynomial value
        """
        return comb(n, i) * (t ** i) * ((1 - t) ** (n - i))

    def bezier_curve(self, points, num_points=100):
        """
        Calculate Bezier curve points.

        Args:
            points: Control points array (n x 2)
            num_points: Number of points to generate

        Returns:
            Array of curve points (num_points x 2)
        """
        n = len(points) - 1
        t = np.linspace(0, 1, num_points)
        curve = np.zeros((num_points, 2))

        for i in range(n + 1):
            curve += np.outer(self.bernstein_poly(i, n, t), points[i])

        return curve

    def calculate_velocity_profile(self, t):
        """
        Calculate velocity-influenced curve (conceptual visualization).

        Args:
            t: Parameter array [0, 1]

        Returns:
            Modified parameter array
        """
        # Simple velocity profile visualization
        # Higher v_init_coeff → slower start
        # Lower v_final_coeff → slower end
        return t ** (1.0 / self.v_init_coeff) * (1 - (1 - t) ** (1.0 / max(0.1, self.v_final_coeff)))

    def setup_plot(self):
        """Setup the interactive plot with sliders."""
        self.fig = plt.figure(figsize=(14, 10))

        # Main plot area
        self.ax = plt.subplot2grid((4, 2), (0, 0), rowspan=3, colspan=2)
        self.ax.set_xlim(-1, 5)
        self.ax.set_ylim(-1, 4)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('Interactive Bezier Curve Visualizer', fontsize=14, fontweight='bold')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')

        # Initial curve plot
        curve = self.bezier_curve(self.control_points, self.num_points)
        self.curve_line, = self.ax.plot(curve[:, 0], curve[:, 1], 'b-', linewidth=2, label='Bezier Curve')

        # Control points and polygon
        self.control_line, = self.ax.plot(
            self.control_points[:, 0],
            self.control_points[:, 1],
            'ro--',
            markersize=10,
            linewidth=1,
            label='Control Points'
        )

        # Add point labels
        self.point_labels = []
        for i, point in enumerate(self.control_points):
            label = self.ax.text(point[0], point[1] + 0.2, f'P{i}',
                                ha='center', fontsize=10, fontweight='bold')
            self.point_labels.append(label)

        self.ax.legend(loc='upper right')

        # Sliders for parameters
        slider_color = 'lightgoldenrodyellow'

        # v_init_coeff slider
        ax_v_init = plt.subplot2grid((4, 2), (3, 0))
        self.slider_v_init = Slider(
            ax_v_init, 'v_init_coeff', 0.1, 2.0, valinit=self.v_init_coeff,
            color=slider_color, valstep=0.1
        )
        self.slider_v_init.on_changed(self.update)

        # v_final_coeff slider
        ax_v_final = plt.subplot2grid((4, 2), (3, 1))
        self.slider_v_final = Slider(
            ax_v_final, 'v_final_coeff', 0.1, 2.0, valinit=self.v_final_coeff,
            color=slider_color, valstep=0.1
        )
        self.slider_v_final.on_changed(self.update)

        # Control point sliders (for first and last points)
        self.point_sliders = []

        plt.tight_layout()

    def update(self, val=None):
        """Update the plot when parameters change."""
        # Update parameters
        self.v_init_coeff = self.slider_v_init.val
        self.v_final_coeff = self.slider_v_final.val

        # Recalculate curve
        curve = self.bezier_curve(self.control_points, self.num_points)

        # Update curve line
        self.curve_line.set_data(curve[:, 0], curve[:, 1])

        # Update control points
        self.control_line.set_data(
            self.control_points[:, 0],
            self.control_points[:, 1]
        )

        # Update labels
        for i, (point, label) in enumerate(zip(self.control_points, self.point_labels)):
            label.set_position((point[0], point[1] + 0.2))

        # Update title with current parameters
        title = (f'Bezier Curve - v_init: {self.v_init_coeff:.1f}, '
                f'v_final: {self.v_final_coeff:.1f}')
        self.ax.set_title(title, fontsize=14, fontweight='bold')

        self.fig.canvas.draw_idle()

    def on_click(self, event):
        """Handle mouse click to move control points."""
        if event.inaxes != self.ax:
            return

        # Find nearest control point
        if event.button == 1:  # Left click
            distances = np.sqrt(np.sum((self.control_points - [event.xdata, event.ydata])**2, axis=1))
            nearest_idx = np.argmin(distances)

            if distances[nearest_idx] < 0.3:  # Close enough to grab
                self.dragging_point = nearest_idx

    def on_motion(self, event):
        """Handle mouse motion for dragging control points."""
        if not hasattr(self, 'dragging_point') or self.dragging_point is None:
            return

        if event.inaxes != self.ax:
            return

        # Update control point position
        self.control_points[self.dragging_point] = [event.xdata, event.ydata]
        self.update()

    def on_release(self, event):
        """Handle mouse release."""
        self.dragging_point = None

    def show(self):
        """Display the interactive plot."""
        # Connect mouse events for dragging control points
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)

        plt.show()


def main():
    """Main function to run the visualizer."""
    print("Bezier Curve Visualizer")
    print("=" * 50)
    print("Controls:")
    print("- Use sliders to adjust velocity coefficients")
    print("- Click and drag red control points to modify curve shape")
    print("- v_init_coeff: Initial velocity coefficient")
    print("- v_final_coeff: Final velocity coefficient")
    print("=" * 50)

    visualizer = BezierCurveVisualizer()
    visualizer.show()


if __name__ == "__main__":
    main()
