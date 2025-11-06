#!/usr/bin/env python3
"""
Autoware Bezier Curve Visualizer

This script reproduces the bezier curve generation logic from autoware_bezier_sampler
with constraints on position, velocity (tangent), curvature, and heading.

Based on:
- autoware_bezier_sampler/src/bezier_sampling.cpp
- autoware_bezier_sampler/src/bezier.cpp
- behavior_path_goal_planner_module/src/pull_over_planner/bezier_pull_over.cpp
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, TextBox, Button
from matplotlib.patches import FancyArrowPatch, Circle
import matplotlib.patches as mpatches


class AutowareBezierCurve:
    """
    Quintic (5th order) Bezier curve implementation matching autoware_bezier_sampler.

    The curve is defined by 6 control points (P0, P1, P2, P3, P4, P5) where:
    - P0, P5: Initial and final positions
    - P1, P4: Depend on initial/final velocities (tangent vectors)
    - P2, P3: Depend on initial/final accelerations (including curvature effects)
    """

    # Quintic Bezier coefficient matrix (from bezier.hpp)
    QUINTIC_COEFFS = np.array([
        [ 1,   0,   0,   0,  0, 0],
        [-5,   5,   0,   0,  0, 0],
        [10, -20,  10,   0,  0, 0],
        [-10,  30, -30,  10,  0, 0],
        [ 5, -20,  30, -20,  5, 0],
        [-1,   5, -10,  10, -5, 1]
    ])

    # Velocity coefficients (derivative of position)
    VELOCITY_COEFFS = np.array([
        QUINTIC_COEFFS[1] * 1,
        QUINTIC_COEFFS[2] * 2,
        QUINTIC_COEFFS[3] * 3,
        QUINTIC_COEFFS[4] * 4,
        QUINTIC_COEFFS[5] * 5
    ])

    # Acceleration coefficients (derivative of velocity)
    ACCEL_COEFFS = np.array([
        VELOCITY_COEFFS[1] * 1,
        VELOCITY_COEFFS[2] * 2,
        VELOCITY_COEFFS[3] * 3,
        VELOCITY_COEFFS[4] * 4
    ])

    def __init__(self, control_points):
        """
        Initialize with 6 control points.

        Args:
            control_points: 6x2 numpy array of control points
        """
        self.control_points = np.array(control_points)
        assert self.control_points.shape == (6, 2), "Need exactly 6 control points"

    def value(self, t):
        """
        Calculate curve position at parameter t using matrix formulation.

        Args:
            t: Parameter value [0, 1]

        Returns:
            2D position (x, y)
        """
        t_vec = np.array([1, t, t**2, t**3, t**4, t**5])
        return t_vec @ self.QUINTIC_COEFFS @ self.control_points

    def velocity(self, t):
        """
        Calculate curve velocity (1st derivative) at parameter t.

        Args:
            t: Parameter value [0, 1]

        Returns:
            2D velocity vector
        """
        t_vec = np.array([1, t, t**2, t**3, t**4])
        return t_vec @ self.VELOCITY_COEFFS @ self.control_points

    def acceleration(self, t):
        """
        Calculate curve acceleration (2nd derivative) at parameter t.

        Args:
            t: Parameter value [0, 1]

        Returns:
            2D acceleration vector
        """
        t_vec = np.array([1, t, t**2, t**3])
        return t_vec @ self.ACCEL_COEFFS @ self.control_points

    def heading(self, t):
        """
        Calculate heading angle at parameter t.

        Args:
            t: Parameter value [0, 1]

        Returns:
            Heading angle in radians
        """
        vel = self.velocity(t)
        return np.arctan2(vel[1], vel[0])

    def curvature(self, t):
        """
        Calculate curvature at parameter t.

        Args:
            t: Parameter value [0, 1]

        Returns:
            Curvature value
        """
        vel = self.velocity(t)
        acc = self.acceleration(t)

        denominator = (vel[0]**2 + vel[1]**2)**(3.0/2.0)
        if denominator == 0:
            return np.inf

        return (vel[0] * acc[1] - acc[0] * vel[1]) / denominator

    def cartesian(self, num_points=100):
        """
        Generate curve points in cartesian coordinates.

        Args:
            num_points: Number of points to generate

        Returns:
            Array of (x, y) coordinates
        """
        t_values = np.linspace(0, 1, num_points)
        return np.array([self.value(t) for t in t_values])

    def cartesian_with_heading(self, num_points=100):
        """
        Generate curve points with heading information.

        Args:
            num_points: Number of points to generate

        Returns:
            Array of (x, y, heading) tuples
        """
        t_values = np.linspace(0, 1, num_points)
        points = []
        for t in t_values:
            pos = self.value(t)
            heading = self.heading(t)
            points.append([pos[0], pos[1], heading])
        return np.array(points)


def generate_bezier_from_states(initial_state, final_state,
                                v_init_coeff, v_final_coeff, acc_coeff):
    """
    Generate a Bezier curve from initial and final states with velocity and acceleration coefficients.

    This matches the logic in bezier_sampling.cpp:sample() and generate() functions.

    Args:
        initial_state: dict with 'position', 'heading', 'curvature'
        final_state: dict with 'position', 'heading', 'curvature'
        v_init_coeff: Initial velocity coefficient (normalized tangent magnitude)
        v_final_coeff: Final velocity coefficient (normalized tangent magnitude)
        acc_coeff: Acceleration coefficient (normalized acceleration magnitude)

    Returns:
        AutowareBezierCurve object
    """
    # Calculate distance between initial and final positions
    initial_pos = np.array(initial_state['position'])
    final_pos = np.array(final_state['position'])
    distance = np.linalg.norm(final_pos - initial_pos)

    # Tangent unit vectors (direction of motion)
    initial_tangent_unit = np.array([
        np.cos(initial_state['heading']),
        np.sin(initial_state['heading'])
    ])
    final_tangent_unit = np.array([
        np.cos(final_state['heading']),
        np.sin(final_state['heading'])
    ])

    # Normal unit vectors (perpendicular to tangent, for curvature)
    initial_normal_unit = np.array([-initial_tangent_unit[1], initial_tangent_unit[0]])
    final_normal_unit = np.array([-final_tangent_unit[1], final_tangent_unit[0]])

    # Calculate tangent lengths (velocity magnitude)
    initial_tangent_length = v_init_coeff * distance
    final_tangent_length = v_final_coeff * distance

    # Calculate acceleration length
    acceleration_length = acc_coeff * distance

    # Calculate initial and final velocity vectors
    initial_velocity = initial_tangent_unit * initial_tangent_length
    final_velocity = final_tangent_unit * final_tangent_length

    # Calculate initial and final acceleration vectors
    # Acceleration includes both tangential and normal (curvature) components
    initial_acceleration = (
        acceleration_length * initial_tangent_unit +
        initial_state['curvature'] * initial_tangent_length**2 * initial_normal_unit
    )
    final_acceleration = (
        acceleration_length * final_tangent_unit +
        final_state['curvature'] * final_tangent_length**2 * final_normal_unit
    )

    # Generate control points from states, velocities, and accelerations
    # This follows the formula in bezier_sampling.cpp:generate()
    control_points = np.zeros((6, 2))

    # P0 and P5: Initial and final positions
    control_points[0] = initial_pos
    control_points[5] = final_pos

    # P1 and P4: Depend on velocities
    control_points[1] = control_points[0] + (1.0 / 5.0) * initial_velocity
    control_points[4] = control_points[5] - (1.0 / 5.0) * final_velocity

    # P2 and P3: Depend on accelerations
    control_points[2] = (2 * control_points[1] - control_points[0] +
                        (1.0 / 20.0) * initial_acceleration)
    control_points[3] = (2 * control_points[4] - control_points[5] -
                        (1.0 / 20.0) * final_acceleration)

    return AutowareBezierCurve(control_points)


class BezierVisualizer:
    """Interactive visualizer for Autoware Bezier curves with state constraints."""

    def __init__(self):
        """Initialize the visualizer with default states."""
        # Initial state
        self.initial_state = {
            'position': [0.0, 0.0],
            'heading': 0.0,  # radians
            'curvature': 0.0
        }

        # Final state
        self.final_state = {
            'position': [10.0, 5.0],
            'heading': 0.2,  # radians
            'curvature': 0.0
        }

        # Bezier parameters (matching bezier_pull_over.cpp:193-202)
        self.v_init_coeff = 0.5
        self.v_final_coeff = 0.1
        self.acc_coeff = 0.0

        self.num_points = 100

        # Setup plot
        self.setup_plot()
        self.update_curve()

    def setup_plot(self):
        """Setup the interactive plot with controls."""
        self.fig = plt.figure(figsize=(16, 10))

        # Main plot area for curve
        self.ax_curve = plt.subplot2grid((6, 3), (0, 0), rowspan=4, colspan=3)
        self.ax_curve.set_aspect('equal')
        self.ax_curve.grid(True, alpha=0.3)
        self.ax_curve.set_title(
            'Autoware Bezier Curve: State-Constrained Quintic Path',
            fontsize=14, fontweight='bold'
        )
        self.ax_curve.set_xlabel('X [m]')
        self.ax_curve.set_ylabel('Y [m]')

        # Curvature plot
        self.ax_curvature = plt.subplot2grid((6, 3), (4, 0), colspan=3)
        self.ax_curvature.set_title('Curvature Profile', fontsize=10)
        self.ax_curvature.set_xlabel('Arc Length Ratio')
        self.ax_curvature.set_ylabel('Curvature [1/m]')
        self.ax_curvature.grid(True, alpha=0.3)

        # Parameter sliders
        slider_color = 'lightgoldenrodyellow'

        # v_init_coeff slider (193-196 in bezier_pull_over.cpp)
        ax_v_init = plt.subplot2grid((6, 3), (5, 0))
        self.slider_v_init = Slider(
            ax_v_init, 'v_init', 0.1, 2.0, valinit=self.v_init_coeff,
            color=slider_color, valstep=0.05
        )
        self.slider_v_init.on_changed(self.on_param_change)

        # v_final_coeff slider (197 in bezier_pull_over.cpp)
        ax_v_final = plt.subplot2grid((6, 3), (5, 1))
        self.slider_v_final = Slider(
            ax_v_final, 'v_final', 0.1, 2.0, valinit=self.v_final_coeff,
            color=slider_color, valstep=0.05
        )
        self.slider_v_final.on_changed(self.on_param_change)

        # acc_coeff slider (198 in bezier_pull_over.cpp)
        ax_acc = plt.subplot2grid((6, 3), (5, 2))
        self.slider_acc = Slider(
            ax_acc, 'acc', 0.0, 10.0, valinit=self.acc_coeff,
            color=slider_color, valstep=0.5
        )
        self.slider_acc.on_changed(self.on_param_change)

        # Initialize plot elements
        self.curve_line, = self.ax_curve.plot([], [], 'b-', linewidth=2, label='Bezier Path')
        self.control_line, = self.ax_curve.plot([], [], 'ro--', markersize=6,
                                                linewidth=1, alpha=0.5, label='Control Points')

        # Arrow patches for state visualization
        self.initial_arrow = None
        self.final_arrow = None

        # Info text
        self.info_text = self.ax_curve.text(
            0.02, 0.98, '', transform=self.ax_curve.transAxes,
            verticalalignment='top', fontsize=9,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5)
        )

        self.ax_curve.legend(loc='upper right')

        plt.tight_layout()

    def update_curve(self):
        """Update the displayed curve based on current parameters."""
        # Generate curve
        self.bezier = generate_bezier_from_states(
            self.initial_state, self.final_state,
            self.v_init_coeff, self.v_final_coeff, self.acc_coeff
        )

        # Get curve points
        points = self.bezier.cartesian(self.num_points)

        # Update curve line
        self.curve_line.set_data(points[:, 0], points[:, 1])

        # Update control points
        cp = self.bezier.control_points
        self.control_line.set_data(cp[:, 0], cp[:, 1])

        # Update state arrows
        self.draw_state_arrow(
            self.initial_state['position'],
            self.initial_state['heading'],
            'green', 'Initial'
        )
        self.draw_state_arrow(
            self.final_state['position'],
            self.final_state['heading'],
            'red', 'Final'
        )

        # Update axis limits
        all_x = np.concatenate([points[:, 0], cp[:, 0]])
        all_y = np.concatenate([points[:, 1], cp[:, 1]])
        margin = 2.0
        self.ax_curve.set_xlim(all_x.min() - margin, all_x.max() + margin)
        self.ax_curve.set_ylim(all_y.min() - margin, all_y.max() + margin)

        # Update curvature plot
        self.update_curvature_plot()

        # Update info text
        self.update_info_text()

        self.fig.canvas.draw_idle()

    def draw_state_arrow(self, position, heading, color, label):
        """Draw an arrow representing a state (position and heading)."""
        arrow_length = 1.5
        dx = arrow_length * np.cos(heading)
        dy = arrow_length * np.sin(heading)

        # Remove old arrow if exists
        if label == 'Initial' and self.initial_arrow:
            self.initial_arrow.remove()
        elif label == 'Final' and self.final_arrow:
            self.final_arrow.remove()

        # Create new arrow
        arrow = FancyArrowPatch(
            position, [position[0] + dx, position[1] + dy],
            arrowstyle='->', mutation_scale=20, linewidth=2,
            color=color, label=label
        )
        self.ax_curve.add_patch(arrow)

        # Add position marker
        circle = Circle(position, 0.2, color=color, zorder=10)
        self.ax_curve.add_patch(circle)

        if label == 'Initial':
            self.initial_arrow = arrow
        else:
            self.final_arrow = arrow

    def update_curvature_plot(self):
        """Update the curvature profile plot."""
        self.ax_curvature.clear()

        # Calculate curvature at multiple points
        t_values = np.linspace(0, 1, self.num_points)
        curvatures = [self.bezier.curvature(t) for t in t_values]

        # Clip extreme values for visualization
        curvatures = np.clip(curvatures, -1.0, 1.0)

        self.ax_curvature.plot(t_values, curvatures, 'b-', linewidth=2)
        self.ax_curvature.axhline(y=0, color='k', linestyle='--', alpha=0.3)

        # Mark initial and final curvatures
        self.ax_curvature.plot(0, self.initial_state['curvature'], 'go',
                              markersize=8, label='Initial')
        self.ax_curvature.plot(1, self.final_state['curvature'], 'ro',
                              markersize=8, label='Final')

        self.ax_curvature.set_title('Curvature Profile', fontsize=10)
        self.ax_curvature.set_xlabel('Arc Length Ratio')
        self.ax_curvature.set_ylabel('Curvature [1/m]')
        self.ax_curvature.grid(True, alpha=0.3)
        self.ax_curvature.legend(loc='upper right')

    def update_info_text(self):
        """Update the information text display."""
        info = (
            f"Initial State:\n"
            f"  Pos: ({self.initial_state['position'][0]:.2f}, "
            f"{self.initial_state['position'][1]:.2f})\n"
            f"  Heading: {np.rad2deg(self.initial_state['heading']):.1f}°\n"
            f"  Curvature: {self.initial_state['curvature']:.4f}\n\n"
            f"Final State:\n"
            f"  Pos: ({self.final_state['position'][0]:.2f}, "
            f"{self.final_state['position'][1]:.2f})\n"
            f"  Heading: {np.rad2deg(self.final_state['heading']):.1f}°\n"
            f"  Curvature: {self.final_state['curvature']:.4f}\n\n"
            f"Parameters:\n"
            f"  v_init_coeff: {self.v_init_coeff:.2f}\n"
            f"  v_final_coeff: {self.v_final_coeff:.2f}\n"
            f"  acc_coeff: {self.acc_coeff:.2f}"
        )
        self.info_text.set_text(info)

    def on_param_change(self, val):
        """Callback when parameter sliders change."""
        self.v_init_coeff = self.slider_v_init.val
        self.v_final_coeff = self.slider_v_final.val
        self.acc_coeff = self.slider_acc.val
        self.update_curve()

    def show(self):
        """Display the interactive visualizer."""
        plt.show()


def main():
    """Main function to run the visualizer."""
    print("=" * 70)
    print("Autoware Bezier Curve Visualizer")
    print("=" * 70)
    print("\nThis visualizer reproduces the bezier curve generation from:")
    print("  - autoware_bezier_sampler (quintic bezier implementation)")
    print("  - behavior_path_goal_planner (pull-over path planning)")
    print("\nFeatures:")
    print("  - State-constrained path: position, heading, curvature")
    print("  - Quintic (5th order) Bezier curves")
    print("  - Interactive parameter control via sliders")
    print("  - Real-time curvature profile visualization")
    print("\nParameters (matching bezier_pull_over.cpp:193-202):")
    print("  - v_init_coeff: Initial velocity coefficient")
    print("    → Controls how quickly steering begins (larger = slower start)")
    print("  - v_final_coeff: Final velocity coefficient")
    print("    → Controls how steering approaches target (smaller = slower end)")
    print("  - acc_coeff: Acceleration coefficient")
    print("    → Controls path smoothness (larger = smoother transitions)")
    print("\nControls:")
    print("  - Adjust sliders to modify curve parameters")
    print("  - Green arrow: Initial state (position & heading)")
    print("  - Red arrow: Final state (position & heading)")
    print("=" * 70)

    visualizer = BezierVisualizer()
    visualizer.show()


if __name__ == "__main__":
    main()
