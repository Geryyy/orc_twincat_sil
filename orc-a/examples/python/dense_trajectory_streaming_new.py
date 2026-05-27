"""
Example: Streaming dense (1kHz) trajectories with feedforward torques for long durations

This example demonstrates how to continuously stream pre-computed trajectory data
to the robot controller for extended periods (e.g., 10 minutes). The trajectory is
sent at 1kHz and upsampled to 8kHz on the real-time controller via linear interpolation.

Data sent per point:
- q (joint positions)
- q_dot (joint velocities)
- tau_ff (feedforward torques / Vorsteuerung)

The feedforward torque (tau_ff) is added to the controller output on the real-time side:
    tau_command = tau_feedback + tau_ff

This is the same mechanism used by move_jointspace with the JointspaceTrajectory,
but allows for pre-computed feedforward torques (e.g., from inverse dynamics or
learned models).

IMPORTANT: Run simulate_iiwa.py first before running this script!
"""

import os
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import orcpy.core as oc
import orcpy.robots as orco

# Optional matplotlib for visualization
try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    MATPLOTLIB_AVAILABLE = True
except ImportError:
    plt = None
    FuncAnimation = None
    MATPLOTLIB_AVAILABLE = False
    print("Note: matplotlib not available. Install it for real-time visualization.")


# ============================================================================
# Trajectory Generator Interface
# ============================================================================


class TrajectoryGenerator:
    """Base class for trajectory generators."""

    def __init__(self, dof: int = 7):
        self.dof = dof

    def get_initial_config(self) -> np.ndarray:
        """Return the initial joint configuration for the trajectory."""
        raise NotImplementedError("Subclasses must implement get_initial_config")

    def get_duration(self) -> float:
        """Return the total duration of the trajectory in seconds."""
        raise NotImplementedError("Subclasses must implement get_duration")

    def generate_chunk(
        self, t_start: float, t_end: float, q_start: np.ndarray, frequency: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate a trajectory chunk from t_start to t_end.

        Args:
            t_start: Start time (seconds, absolute robot time)
            t_end: End time (seconds, absolute robot time)
            q_start: Starting joint configuration
            frequency: Sample frequency (Hz)

        Returns:
            Tuple of (time_pts, q_pts, q_dot_pts, q_dotdot_pts, tau_ff_pts)
            - time_pts: [N] array of time stamps
            - q_pts: [N x DOF] joint positions
            - q_dot_pts: [N x DOF] joint velocities
            - q_dotdot_pts: [N x DOF] joint accelerations
            - tau_ff_pts: [N x DOF] feedforward torques
        """
        raise NotImplementedError("Subclasses must implement generate_chunk")

    def get_state_at_time(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get trajectory state at a specific time (for visualization).

        This method should NOT modify internal state.

        Args:
            t: Absolute robot time

        Returns:
            Tuple of (q, q_dot, tau_ff) at time t
        """
        # Default implementation returns zeros - subclasses should override
        return np.zeros(self.dof), np.zeros(self.dof), np.zeros(self.dof)


class SinusoidalTrajectoryGenerator(TrajectoryGenerator):
    """Simple sinusoidal trajectory for demonstration and testing."""

    def __init__(
        self,
        dof: int = 7,
        amplitude: float = 0.3,
        frequency_hz: float = 0.05,
        joint_index: int = 0,
        duration: float = 600.0,
    ):
        """
        Args:
            dof: Degrees of freedom
            amplitude: Motion amplitude in radians (default 0.3)
            frequency_hz: Oscillation frequency in Hz (default 0.05 = 20s period)
            joint_index: Which joint to move (default 0)
            duration: Total trajectory duration in seconds (default 600 = 10 minutes)
        """
        super().__init__(dof)
        self.amplitude = amplitude
        self.omega = 2 * np.pi * frequency_hz
        self.joint_index = joint_index
        self.duration = duration
        self.q_base = None  # Will be set on first call
        self._t_offset = 0.0
        self._initialized = False

    def get_initial_config(self) -> np.ndarray:
        if self.q_base is not None:
            return self.q_base.copy()
        return np.zeros(self.dof)

    def get_duration(self) -> float:
        return self.duration

    def generate_chunk(
        self, t_start: float, t_end: float, q_start: np.ndarray, frequency: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        # Initialize on first call
        if not self._initialized:
            self.q_base = q_start.copy()
            self._t_offset = t_start
            self._initialized = True

        dt = 1.0 / frequency
        n_points = max(1, int((t_end - t_start) * frequency))

        t_vec = np.linspace(t_start, t_end, n_points, endpoint=False)
        traj_t = t_vec - self._t_offset  # Trajectory-relative time

        q_pts = np.zeros((n_points, self.dof))
        q_dot_pts = np.zeros((n_points, self.dof))
        q_dotdot_pts = np.zeros((n_points, self.dof))
        tau_ff_pts = np.zeros((n_points, self.dof))

        for i, t in enumerate(traj_t):
            # Position
            q_pts[i] = self.q_base.copy()
            q_pts[i, self.joint_index] += self.amplitude * np.sin(self.omega * t)

            # Velocity
            q_dot_pts[i, self.joint_index] = self.amplitude * self.omega * np.cos(self.omega * t)

            # Acceleration
            q_dotdot_pts[i, self.joint_index] = (
                -self.amplitude * self.omega**2 * np.sin(self.omega * t)
            )

            # Feedforward torque: simple inertia-based estimate
            # tau_ff = M * q_ddot (simplified, real implementation would use inverse dynamics)
            # Here we use a simple approximation based on joint inertias
            tau_ff_pts[i, self.joint_index] = 0.5 * q_dotdot_pts[i, self.joint_index]

        return t_vec, q_pts, q_dot_pts, q_dotdot_pts, tau_ff_pts


class LoopingTrajectoryGenerator(TrajectoryGenerator):
    """
    Trajectory that loops between two configurations (e.g., zeros to ones).

    Uses quintic polynomial for smooth motion (zero velocity/acceleration at endpoints).
    Loops N times per minute with tau_ff = 0.
    """

    def __init__(
        self,
        dof: int = 7,
        q_start: np.ndarray = None,
        q_end: np.ndarray = None,
        loops_per_minute: float = 5.0,
        duration: float = 600.0,
    ):
        """
        Args:
            dof: Degrees of freedom
            q_start: Starting joint configuration (default: zeros)
            q_end: Ending joint configuration (default: ones)
            loops_per_minute: Number of complete cycles per minute (default: 5)
            duration: Total trajectory duration in seconds (default: 600 = 10 minutes)
        """
        super().__init__(dof)

        self.q_start_cfg = q_start if q_start is not None else np.zeros(dof)
        self.q_end_cfg = q_end if q_end is not None else np.ones(dof)
        self.loops_per_minute = loops_per_minute
        self.duration = duration

        # One complete loop = go to q_end + return to q_start
        # Period = 60s / loops_per_minute
        self.period = 60.0 / loops_per_minute  # seconds per complete loop
        self.half_period = self.period / 2.0  # seconds for one-way motion

        self._t_offset = 0.0
        self._initialized = False

        print("LoopingTrajectoryGenerator:")
        print(f"  q_start: {self.q_start_cfg}")
        print(f"  q_end:   {self.q_end_cfg}")
        print(f"  Loops per minute: {loops_per_minute}")
        print(f"  Period: {self.period:.2f}s (half: {self.half_period:.2f}s)")
        print(f"  Total loops in {duration / 60:.1f} min: {duration / self.period:.1f}")

    def get_initial_config(self) -> np.ndarray:
        return self.q_start_cfg.copy()

    def get_duration(self) -> float:
        return self.duration

    def _quintic_blend(self, tau: float) -> tuple[float, float, float]:
        """
        Quintic polynomial blend with zero velocity and acceleration at endpoints.

        Args:
            tau: Normalized time [0, 1]

        Returns:
            (s, s_dot, s_dotdot): Position, velocity, acceleration blend factors
        """
        tau = np.clip(tau, 0.0, 1.0)
        tau2 = tau * tau
        tau3 = tau2 * tau
        tau4 = tau3 * tau
        tau5 = tau4 * tau

        # Quintic polynomial: s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
        # This gives s(0)=0, s(1)=1, s'(0)=s'(1)=0, s''(0)=s''(1)=0
        s = 10 * tau3 - 15 * tau4 + 6 * tau5
        s_dot = 30 * tau2 - 60 * tau3 + 30 * tau4  # derivative w.r.t. tau
        s_dotdot = 60 * tau - 180 * tau2 + 120 * tau3  # second derivative w.r.t. tau

        return s, s_dot, s_dotdot

    def generate_chunk(
        self, t_start: float, t_end: float, q_start: np.ndarray, frequency: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if not self._initialized:
            self._t_offset = t_start
            self._initialized = True

        n_points = max(1, int((t_end - t_start) * frequency))
        t_vec = np.linspace(t_start, t_end, n_points, endpoint=False)
        traj_t = t_vec - self._t_offset  # Trajectory-relative time

        q_pts = np.zeros((n_points, self.dof))
        q_dot_pts = np.zeros((n_points, self.dof))
        q_dotdot_pts = np.zeros((n_points, self.dof))
        tau_ff_pts = np.zeros((n_points, self.dof))  # Always zero as requested

        delta_q = self.q_end_cfg - self.q_start_cfg

        for i, t in enumerate(traj_t):
            # Determine where we are in the current loop
            t_in_period = t % self.period

            if t_in_period < self.half_period:
                # First half: moving from q_start to q_end
                tau = t_in_period / self.half_period
                q0, q1 = self.q_start_cfg, self.q_end_cfg
            else:
                # Second half: moving from q_end to q_start
                tau = (t_in_period - self.half_period) / self.half_period
                q0, q1 = self.q_end_cfg, self.q_start_cfg

            s, s_dot_tau, s_dotdot_tau = self._quintic_blend(tau)

            # Convert derivatives from d/d(tau) to d/dt
            # tau = t / T, so d/dt = (1/T) * d/d(tau)
            T = self.half_period
            s_dot = s_dot_tau / T
            s_dotdot = s_dotdot_tau / (T * T)

            # Interpolate position, velocity, acceleration
            q_pts[i] = q0 + s * (q1 - q0)
            q_dot_pts[i] = s_dot * (q1 - q0)
            q_dotdot_pts[i] = s_dotdot * (q1 - q0)

            # tau_ff stays zero

        return t_vec, q_pts, q_dot_pts, q_dotdot_pts, tau_ff_pts

    def get_state_at_time(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get trajectory state at a specific time (for visualization).
        Does NOT modify internal state.
        """
        if not self._initialized:
            # Not started yet
            return self.q_start_cfg.copy(), np.zeros(self.dof), np.zeros(self.dof)

        # Convert absolute time to trajectory-relative time
        traj_t = t - self._t_offset

        # Clamp to valid range
        if traj_t < 0:
            return self.q_start_cfg.copy(), np.zeros(self.dof), np.zeros(self.dof)
        if traj_t > self.duration:
            return self.q_start_cfg.copy(), np.zeros(self.dof), np.zeros(self.dof)

        # Determine where we are in the current loop
        t_in_period = traj_t % self.period

        if t_in_period < self.half_period:
            # First half: moving from q_start to q_end
            tau = t_in_period / self.half_period
            q0, q1 = self.q_start_cfg, self.q_end_cfg
        else:
            # Second half: moving from q_end to q_start
            tau = (t_in_period - self.half_period) / self.half_period
            q0, q1 = self.q_end_cfg, self.q_start_cfg

        s, s_dot_tau, _ = self._quintic_blend(tau)

        # Convert derivatives from d/d(tau) to d/dt
        T = self.half_period
        s_dot = s_dot_tau / T

        # Compute state
        q = q0 + s * (q1 - q0)
        q_dot = s_dot * (q1 - q0)
        tau_ff = np.zeros(self.dof)  # Always zero for this generator

        return q, q_dot, tau_ff


class NPZTrajectoryGenerator(TrajectoryGenerator):
    """
    Trajectory generator that loads pre-computed trajectory from NPZ file.

    Expected NPZ fields:
    - time or t: time stamps [N] (at source_frequency, e.g., 1kHz)
    - q: joint positions [N x DOF]
    - qdot or q_dot: joint velocities [N x DOF]
    - tau_ff or tau_ext: feedforward/external torques [N x DOF]

    Optional:
    - qddot or q_ddot: joint accelerations [N x DOF] (computed if not present)
    """

    def __init__(self, npz_path: str, source_frequency: float = 1000, dof: int = 7):
        """
        Args:
            npz_path: Path to the NPZ file
            source_frequency: Sample rate of the data in the NPZ file (Hz)
            dof: Degrees of freedom
        """
        super().__init__(dof)

        print(f"Loading trajectory from: {npz_path}")
        data = np.load(npz_path)

        # Load time data
        if "time" in data:
            self.time_data = data["time"].flatten()
        elif "t" in data:
            self.time_data = data["t"].flatten()
        else:
            # Create artificial time array
            n_points = len(data["q"])
            self.time_data = np.arange(n_points) / source_frequency
            print("  No time data found - created from sample count")

        # Load position data
        self.q_data = np.array(data["q"])
        if self.q_data.ndim == 1:
            self.q_data = self.q_data.reshape(-1, 1)

        # Load velocity data
        if "qdot" in data:
            self.qdot_data = np.array(data["qdot"])
        elif "q_dot" in data:
            self.qdot_data = np.array(data["q_dot"])
        else:
            print("  No velocity data found - computing from positions")
            self.qdot_data = np.gradient(self.q_data, 1.0 / source_frequency, axis=0)

        if self.qdot_data.ndim == 1:
            self.qdot_data = self.qdot_data.reshape(-1, 1)

        # Load feedforward torque data
        if "tau_ff" in data:
            self.tau_ff_data = np.array(data["tau_ff"])
        elif "tau_ext" in data:
            self.tau_ff_data = np.array(data["tau_ext"])
        else:
            print("  No feedforward torque data found - using zeros")
            self.tau_ff_data = np.zeros_like(self.q_data)

        if self.tau_ff_data.ndim == 1:
            self.tau_ff_data = self.tau_ff_data.reshape(-1, 1)

        # Load or compute acceleration data
        if "qddot" in data:
            self.qddot_data = np.array(data["qddot"])
        elif "q_ddot" in data:
            self.qddot_data = np.array(data["q_ddot"])
        else:
            print("  No acceleration data found - computing from velocities")
            self.qddot_data = np.gradient(self.qdot_data, 1.0 / source_frequency, axis=0)

        if self.qddot_data.ndim == 1:
            self.qddot_data = self.qddot_data.reshape(-1, 1)

        self.source_frequency = source_frequency

        # Normalize time to start from 0
        self.traj_time = self.time_data - self.time_data[0]
        self.total_duration = self.traj_time[-1]

        self._t_offset = 0.0
        self._initialized = False

        print(
            f"  Loaded {len(self.time_data)} points ({self.total_duration:.1f}s at {source_frequency}Hz)"
        )
        print(f"  q shape: {self.q_data.shape}")
        print(f"  qdot shape: {self.qdot_data.shape}")
        print(f"  tau_ff shape: {self.tau_ff_data.shape}")

    def get_initial_config(self) -> np.ndarray:
        return self.q_data[0].copy()

    def get_duration(self) -> float:
        return self.total_duration

    def generate_chunk(
        self, t_start: float, t_end: float, q_start: np.ndarray, frequency: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        # Initialize time offset on first call
        if not self._initialized:
            self._t_offset = t_start
            self._initialized = True

        # Generate output time vector at target frequency
        n_output_points = max(1, int((t_end - t_start) * frequency))
        t_vec = np.linspace(t_start, t_end, n_output_points, endpoint=False)
        traj_t_vec = t_vec - self._t_offset  # Trajectory-relative times

        # Clamp to valid range
        traj_t_vec = np.clip(traj_t_vec, 0, self.total_duration - 1e-9)

        # Linear interpolation for each joint
        q_pts = np.zeros((n_output_points, self.dof))
        qdot_pts = np.zeros((n_output_points, self.dof))
        qddot_pts = np.zeros((n_output_points, self.dof))
        tau_ff_pts = np.zeros((n_output_points, self.dof))

        for j in range(self.dof):
            q_pts[:, j] = np.interp(traj_t_vec, self.traj_time, self.q_data[:, j])
            qdot_pts[:, j] = np.interp(traj_t_vec, self.traj_time, self.qdot_data[:, j])
            qddot_pts[:, j] = np.interp(traj_t_vec, self.traj_time, self.qddot_data[:, j])
            tau_ff_pts[:, j] = np.interp(traj_t_vec, self.traj_time, self.tau_ff_data[:, j])

        return t_vec, q_pts, qdot_pts, qddot_pts, tau_ff_pts


class SplineTrajectoryGenerator(TrajectoryGenerator):
    """
    Quintic spline trajectory generator between waypoints.

    Ensures smooth position, velocity, and acceleration (C2 continuity).
    """

    def __init__(
        self, waypoints: list, durations: list, dof: int = 7, tau_ff_waypoints: list = None
    ):
        """
        Args:
            waypoints: List of joint configurations [q0, q1, q2, ...] as numpy arrays
            durations: List of durations between waypoints [d01, d12, ...] in seconds
            dof: Degrees of freedom
            tau_ff_waypoints: Optional list of feedforward torques at each waypoint
        """
        super().__init__(dof)

        assert len(waypoints) >= 2, "Need at least 2 waypoints"
        assert len(durations) == len(waypoints) - 1, "Need N-1 durations for N waypoints"

        self.waypoints = [np.array(wp).flatten() for wp in waypoints]
        self.durations = durations

        # Feedforward torques at waypoints (for interpolation)
        if tau_ff_waypoints is not None:
            self.tau_ff_waypoints = [np.array(t).flatten() for t in tau_ff_waypoints]
        else:
            self.tau_ff_waypoints = [np.zeros(dof) for _ in waypoints]

        # Compute cumulative times
        self.waypoint_times = [0.0]
        for d in durations:
            self.waypoint_times.append(self.waypoint_times[-1] + d)

        self.total_duration = self.waypoint_times[-1]
        self._t_offset = 0.0
        self._initialized = False

        print("SplineTrajectoryGenerator created:")
        print(f"  {len(waypoints)} waypoints, total duration: {self.total_duration:.1f}s")

    def get_initial_config(self) -> np.ndarray:
        return self.waypoints[0].copy()

    def get_duration(self) -> float:
        return self.total_duration

    def _quintic_spline(
        self,
        t: float,
        t0: float,
        t1: float,
        q0: np.ndarray,
        q1: np.ndarray,
        v0: np.ndarray,
        v1: np.ndarray,
        a0: np.ndarray,
        a1: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Compute quintic polynomial spline at time t."""
        T = t1 - t0
        if T <= 0:
            return q0.copy(), v0.copy(), a0.copy()

        tau = np.clip((t - t0) / T, 0.0, 1.0)

        # Quintic polynomial coefficients
        c0 = q0
        c1 = v0 * T
        c2 = 0.5 * a0 * T**2
        c3 = 10 * (q1 - q0) - 6 * v0 * T - 4 * v1 * T - 1.5 * a0 * T**2 + 0.5 * a1 * T**2
        c4 = -15 * (q1 - q0) + 8 * v0 * T + 7 * v1 * T + 1.5 * a0 * T**2 - a1 * T**2
        c5 = 6 * (q1 - q0) - 3 * v0 * T - 3 * v1 * T - 0.5 * a0 * T**2 + 0.5 * a1 * T**2

        tau2, tau3, tau4, tau5 = tau**2, tau**3, tau**4, tau**5

        q = c0 + c1 * tau + c2 * tau2 + c3 * tau3 + c4 * tau4 + c5 * tau5
        q_dot = (c1 + 2 * c2 * tau + 3 * c3 * tau2 + 4 * c4 * tau3 + 5 * c5 * tau4) / T
        q_dotdot = (2 * c2 + 6 * c3 * tau + 12 * c4 * tau2 + 20 * c5 * tau3) / (T**2)

        return q, q_dot, q_dotdot

    def _get_segment(self, traj_time: float) -> int:
        """Find which segment the trajectory time falls into."""
        for i in range(len(self.waypoint_times) - 1):
            if traj_time < self.waypoint_times[i + 1]:
                return i
        return len(self.waypoint_times) - 2

    def generate_chunk(
        self, t_start: float, t_end: float, q_start: np.ndarray, frequency: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if not self._initialized:
            self._t_offset = t_start
            self._initialized = True

        n_output_points = max(1, int((t_end - t_start) * frequency))
        t_vec = np.linspace(t_start, t_end, n_output_points, endpoint=False)
        traj_t_vec = np.clip(t_vec - self._t_offset, 0, self.total_duration - 1e-9)

        q_pts = np.zeros((n_output_points, self.dof))
        qdot_pts = np.zeros((n_output_points, self.dof))
        qddot_pts = np.zeros((n_output_points, self.dof))
        tau_ff_pts = np.zeros((n_output_points, self.dof))

        for i, traj_t in enumerate(traj_t_vec):
            seg = self._get_segment(traj_t)

            t0 = self.waypoint_times[seg]
            t1 = self.waypoint_times[seg + 1]
            q0 = self.waypoints[seg]
            q1 = self.waypoints[seg + 1]
            tau0 = self.tau_ff_waypoints[seg]
            tau1 = self.tau_ff_waypoints[seg + 1]

            # Zero velocity/acceleration at waypoints for smooth stop-and-go
            v0, v1 = np.zeros(self.dof), np.zeros(self.dof)
            a0, a1 = np.zeros(self.dof), np.zeros(self.dof)

            q, qd, qdd = self._quintic_spline(traj_t, t0, t1, q0, q1, v0, v1, a0, a1)
            q_pts[i] = q
            qdot_pts[i] = qd
            qddot_pts[i] = qdd

            # Linear interpolation for feedforward torques
            alpha = (traj_t - t0) / (t1 - t0) if t1 > t0 else 0
            tau_ff_pts[i] = tau0 + alpha * (tau1 - tau0)

        return t_vec, q_pts, qdot_pts, qddot_pts, tau_ff_pts


# ============================================================================
# Real-Time Buffer Visualization
# ============================================================================


class BufferVisualization:
    """
    Real-time visualization of trajectory buffer and robot state.

    Shows:
    - Rolling window of sent trajectory (q, q_dot, tau_ff)
    - Actual robot position/velocity overlaid
    - Buffer status indicator

    This is a Python-side visualization. For TwinCAT HMI, you would need
    to expose buffer data via PLC variables and use TcHmiScope or TcHmiTrendLineChart.
    """

    def __init__(
        self,
        dof: int = 7,
        window_seconds: float = 10.0,
        update_interval_ms: int = 100,
        joints_to_plot: list = None,
    ):
        """
        Args:
            dof: Degrees of freedom
            window_seconds: How many seconds of data to show (default 10s)
            update_interval_ms: Update interval in milliseconds (default 100ms = 10Hz)
            joints_to_plot: Which joints to plot (default [0, 1, 2] = first 3 joints)
        """
        if not MATPLOTLIB_AVAILABLE:
            raise ImportError("matplotlib is required for visualization")

        self.dof = dof
        self.window_seconds = window_seconds
        self.update_interval_ms = update_interval_ms
        self.joints_to_plot = joints_to_plot if joints_to_plot is not None else [0, 1, 2]

        # Data buffers (store recent history)
        # We store at 10Hz for visualization (100ms intervals)
        self.max_points = int(window_seconds * (1000 / update_interval_ms))

        # Sent trajectory data (what we sent to robot)
        self.sent_time = deque(maxlen=self.max_points)
        self.sent_q = {j: deque(maxlen=self.max_points) for j in self.joints_to_plot}
        self.sent_q_dot = {j: deque(maxlen=self.max_points) for j in self.joints_to_plot}
        self.sent_tau_ff = {j: deque(maxlen=self.max_points) for j in self.joints_to_plot}

        # Actual robot state
        self.actual_time = deque(maxlen=self.max_points)
        self.actual_q = {j: deque(maxlen=self.max_points) for j in self.joints_to_plot}
        self.actual_q_dot = {j: deque(maxlen=self.max_points) for j in self.joints_to_plot}

        # Buffer info
        self.buffer_time_history = deque(maxlen=self.max_points)
        self.buffer_values = deque(maxlen=self.max_points)

        # Threading
        self._lock = threading.Lock()
        self._running = False
        self._fig = None
        self._ani = None

        # Colors for joints
        self.colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b", "#e377c2"]

    def add_sent_data(
        self, robot_time: float, q: np.ndarray, q_dot: np.ndarray, tau_ff: np.ndarray
    ):
        """Add sent trajectory data point (call periodically, e.g., every 100ms)."""
        with self._lock:
            self.sent_time.append(robot_time)
            for j in self.joints_to_plot:
                self.sent_q[j].append(q[j] if j < len(q) else 0)
                self.sent_q_dot[j].append(q_dot[j] if j < len(q_dot) else 0)
                self.sent_tau_ff[j].append(tau_ff[j] if j < len(tau_ff) else 0)

    def add_actual_data(self, robot_time: float, q: np.ndarray, q_dot: np.ndarray):
        """Add actual robot state (call periodically, e.g., every 100ms)."""
        with self._lock:
            self.actual_time.append(robot_time)
            for j in self.joints_to_plot:
                self.actual_q[j].append(q[j] if j < len(q) else 0)
                self.actual_q_dot[j].append(q_dot[j] if j < len(q_dot) else 0)

    def add_buffer_info(self, robot_time: float, buffer_seconds: float):
        """Add buffer status (call periodically)."""
        with self._lock:
            self.buffer_time_history.append(robot_time)
            self.buffer_values.append(buffer_seconds)

    def _init_plot(self):
        """Initialize the matplotlib figure."""
        plt.ion()  # Interactive mode
        self._fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        self._fig.suptitle("Dense Trajectory Streaming - Buffer Visualization", fontsize=14)

        self._ax_q = axes[0]
        self._ax_q_dot = axes[1]
        self._ax_tau_ff = axes[2]
        self._ax_buffer = axes[3]

        # Initialize line objects for each joint
        self._lines_q_sent = {}
        self._lines_q_actual = {}
        self._lines_q_dot_sent = {}
        self._lines_q_dot_actual = {}
        self._lines_tau_ff = {}

        for j in self.joints_to_plot:
            color = self.colors[j % len(self.colors)]
            # Sent trajectory (solid line)
            (line,) = self._ax_q.plot(
                [], [], "-", color=color, linewidth=1.5, label=f"J{j + 1} sent", alpha=0.7
            )
            self._lines_q_sent[j] = line
            # Actual position (dashed line, thicker)
            (line,) = self._ax_q.plot(
                [], [], "--", color=color, linewidth=2.5, label=f"J{j + 1} actual"
            )
            self._lines_q_actual[j] = line

            # q_dot
            (line,) = self._ax_q_dot.plot([], [], "-", color=color, linewidth=1.5, alpha=0.7)
            self._lines_q_dot_sent[j] = line
            (line,) = self._ax_q_dot.plot([], [], "--", color=color, linewidth=2.5)
            self._lines_q_dot_actual[j] = line

            # tau_ff
            (line,) = self._ax_tau_ff.plot([], [], "-", color=color, linewidth=1.5)
            self._lines_tau_ff[j] = line

        # Buffer line
        (self._line_buffer,) = self._ax_buffer.plot([], [], "b-", linewidth=2)
        self._ax_buffer.axhline(y=0, color="r", linestyle="--", linewidth=1, label="Zero buffer")

        # Labels
        self._ax_q.set_ylabel("Position [rad]")
        self._ax_q.legend(loc="upper left", fontsize=8, ncol=len(self.joints_to_plot))
        self._ax_q.grid(True, alpha=0.3)

        self._ax_q_dot.set_ylabel("Velocity [rad/s]")
        self._ax_q_dot.grid(True, alpha=0.3)

        self._ax_tau_ff.set_ylabel("tau_ff [Nm]")
        self._ax_tau_ff.grid(True, alpha=0.3)

        self._ax_buffer.set_ylabel("Buffer [s]")
        self._ax_buffer.set_xlabel("Robot Time [s]")
        self._ax_buffer.grid(True, alpha=0.3)
        self._ax_buffer.set_ylim(-5, 35)

        plt.tight_layout()

    def _update_plot(self, frame):
        """Update function for animation."""
        with self._lock:
            if len(self.sent_time) < 2:
                return

            # Get current time range
            t_arr = np.array(self.sent_time)
            t_min = t_arr[-1] - self.window_seconds if len(t_arr) > 0 else 0
            t_max = t_arr[-1] if len(t_arr) > 0 else 10

            # Update q lines
            for j in self.joints_to_plot:
                q_sent = np.array(self.sent_q[j])
                self._lines_q_sent[j].set_data(t_arr, q_sent)

                if len(self.actual_time) > 0:
                    t_actual = np.array(self.actual_time)
                    q_actual = np.array(self.actual_q[j])
                    self._lines_q_actual[j].set_data(t_actual, q_actual)

                # q_dot
                q_dot_sent = np.array(self.sent_q_dot[j])
                self._lines_q_dot_sent[j].set_data(t_arr, q_dot_sent)

                if len(self.actual_time) > 0:
                    q_dot_actual = np.array(self.actual_q_dot[j])
                    self._lines_q_dot_actual[j].set_data(t_actual, q_dot_actual)

                # tau_ff
                tau_ff = np.array(self.sent_tau_ff[j])
                self._lines_tau_ff[j].set_data(t_arr, tau_ff)

            # Update buffer line
            if len(self.buffer_time_history) > 0:
                t_buf = np.array(self.buffer_time_history)
                buf_vals = np.array(self.buffer_values)
                self._line_buffer.set_data(t_buf, buf_vals)

            # Update x limits
            self._ax_q.set_xlim(t_min, t_max)

            # Auto-scale y for q and q_dot
            if len(self.sent_q[self.joints_to_plot[0]]) > 0:
                all_q = np.concatenate([np.array(self.sent_q[j]) for j in self.joints_to_plot])
                q_min, q_max = all_q.min(), all_q.max()
                margin = max(0.1, (q_max - q_min) * 0.1)
                self._ax_q.set_ylim(q_min - margin, q_max + margin)

                all_qd = np.concatenate([np.array(self.sent_q_dot[j]) for j in self.joints_to_plot])
                qd_min, qd_max = all_qd.min(), all_qd.max()
                margin_qd = max(0.1, (qd_max - qd_min) * 0.1)
                self._ax_q_dot.set_ylim(qd_min - margin_qd, qd_max + margin_qd)

                all_tau = np.concatenate(
                    [np.array(self.sent_tau_ff[j]) for j in self.joints_to_plot]
                )
                tau_min, tau_max = all_tau.min(), all_tau.max()
                margin_tau = max(0.1, abs(tau_max - tau_min) * 0.1 + 0.01)
                self._ax_tau_ff.set_ylim(tau_min - margin_tau, tau_max + margin_tau)

        return (
            list(self._lines_q_sent.values())
            + list(self._lines_q_actual.values())
            + list(self._lines_q_dot_sent.values())
            + list(self._lines_q_dot_actual.values())
            + list(self._lines_tau_ff.values())
            + [self._line_buffer]
        )

    def start(self):
        """Start the visualization (non-blocking)."""
        if not MATPLOTLIB_AVAILABLE:
            print("Cannot start visualization: matplotlib not available")
            return

        self._running = True
        self._init_plot()

        # Use FuncAnimation for smooth updates
        self._ani = FuncAnimation(
            self._fig,
            self._update_plot,
            interval=self.update_interval_ms,
            blit=False,
            cache_frame_data=False,
        )
        plt.show(block=False)
        plt.pause(0.1)

    def stop(self):
        """Stop the visualization."""
        self._running = False
        if self._ani:
            self._ani.event_source.stop()
        plt.close(self._fig)

    def update(self):
        """Call this periodically to update the plot (if not using FuncAnimation)."""
        if self._fig:
            self._fig.canvas.draw_idle()
            self._fig.canvas.flush_events()


# ============================================================================
# Dense Trajectory Streamer
# ============================================================================


class DenseTrajectoryStreamer:
    """
    Streams dense trajectory chunks to the robot controller.

    This class handles the timing and buffering required to continuously
    send trajectory data without gaps. The trajectory queue on the robot
    handles buffering and the real-time controller upsamples to 8kHz
    using linear interpolation.

    The feedforward torque (tau_ff) is added to the controller output:
        tau_command = tau_feedback + tau_ff

    This implements "Vorsteuerung" (feedforward control).
    """

    def __init__(
        self,
        robot: orco.Iiwa,
        generator: TrajectoryGenerator,
        frequency: float = 1000,
        chunk_duration: float = 1.0,
        max_buffer_duration: float = 30.0,
        visualization: BufferVisualization = None,
    ):
        """
        Args:
            robot: The robot instance to send trajectories to
            generator: TrajectoryGenerator that produces trajectory chunks
            frequency: Trajectory sample frequency in Hz (default 1000)
            chunk_duration: Duration of each chunk in seconds (default 1.0)
            max_buffer_duration: Maximum seconds of trajectory to buffer ahead (default 30.0)
            visualization: Optional BufferVisualization instance for real-time plotting
        """
        self.robot = robot
        self.generator = generator
        self.frequency = frequency
        self.chunk_duration = chunk_duration
        self.max_buffer_duration = max_buffer_duration
        self.visualization = visualization

        self._running = False
        self._thread: threading.Thread | None = None
        self._next_chunk_start = 0.0
        self._total_duration = 0.0
        self._chunks_sent = 0
        self._trajectory_start_time = 0.0
        self._points_sent = 0
        self._bytes_sent = 0

        # Store last sent trajectory point for visualization
        self._last_sent_q = None
        self._last_sent_q_dot = None
        self._last_sent_tau_ff = None

    def start(self, duration: float = None):
        """
        Start streaming trajectory.

        Args:
            duration: Total trajectory duration in seconds (optional, uses generator's duration if not specified)
        """
        if self._running:
            print("Streamer already running!")
            return

        if duration is None:
            duration = self.generator.get_duration()

        self._total_duration = duration
        self._running = True
        self._chunks_sent = 0
        self._points_sent = 0
        self._bytes_sent = 0

        # Get current robot state
        current_time = self.robot.time.to_sec()
        q_start = self.robot.state.q_act.copy()

        # Start trajectory slightly in the future
        initial_delay = 0.5
        self._trajectory_start_time = current_time + initial_delay
        self._next_chunk_start = self._trajectory_start_time
        trajectory_end_time = self._trajectory_start_time + self._total_duration

        print(f"\n{'=' * 70}")
        print("Starting trajectory stream")
        print(f"{'=' * 70}")
        print(f"  Duration: {duration:.1f}s ({duration / 60:.1f} minutes)")
        print(f"  Frequency: {self.frequency}Hz (upsampled to 8kHz on real-time side)")
        print(
            f"  Chunk size: {self.chunk_duration:.2f}s ({int(self.chunk_duration * self.frequency)} points)"
        )
        print(f"  Max buffer: {self.max_buffer_duration:.1f}s ahead")
        print(f"  Trajectory start time: {self._trajectory_start_time:.3f}s")
        print(f"  Total points to send: {int(duration * self.frequency)}")
        print("")
        print("  DATA BREAKDOWN (per 1kHz sample point):")
        print("    - time:     8 bytes (1 double)")
        print("    - q:       56 bytes (7 doubles)")
        print("    - q_dot:   56 bytes (7 doubles)")
        print("    - q_dotdot:56 bytes (7 doubles)")
        print("    - tau_ff:  56 bytes (7 doubles)")
        print("    TOTAL:    232 bytes per point")
        print("")
        total_bytes = int(duration * self.frequency) * 232
        print(f"  TOTAL DATA for {duration / 60:.0f} min trajectory:")
        print(f"    - Points: {int(duration * self.frequency):,}")
        print(f"    - Size:   {total_bytes / 1e6:.2f} MB")
        print(f"    - Rate:   {self.frequency * 232 / 1e3:.1f} KB/s")
        print(f"{'=' * 70}")
        print("")
        print("  WHAT 'Buffer: 31s' MEANS:")
        print("  -------------------------")
        print("  The trajectory queue on the real-time hardware has received")
        print("  trajectory points that extend 31 seconds INTO THE FUTURE.")
        print("  ")
        print("  Example: If robot time is t=10s and buffer shows 31s, then")
        print("  the hardware has trajectory data for t=10s through t=41s.")
        print("  ")
        print("  This buffer ensures continuous motion even if Python/UDP")
        print("  experiences brief delays. The real-time controller reads")
        print("  from this queue at 8kHz, interpolating between 1kHz points.")
        print(f"{'=' * 70}\n")

        # Pre-buffer chunks up to max_buffer_duration
        print("Pre-buffering...")
        prebuffer_end = min(
            self._trajectory_start_time + self.max_buffer_duration, trajectory_end_time
        )

        while self._next_chunk_start < prebuffer_end and self._running:
            chunk_end = min(self._next_chunk_start + self.chunk_duration, trajectory_end_time)
            if not self._send_chunk(q_start, chunk_end):
                print("ERROR: Failed to send chunk during pre-buffering!")
                self._running = False
                return

        buffer_time = self._next_chunk_start - current_time
        print(f"  Pre-buffered {self._chunks_sent} chunks ({buffer_time:.1f}s ahead)")
        print(f"  Points sent: {self._points_sent}")

        # Start streaming thread
        self._thread = threading.Thread(
            target=self._stream_loop, args=(q_start, trajectory_end_time)
        )
        self._thread.daemon = True
        self._thread.start()

    def _send_chunk(self, q_start: np.ndarray, chunk_end: float) -> bool:
        """Generate and send a single chunk. Returns True on success."""
        try:
            # Generate trajectory chunk
            t_vec, q_pts, q_dot_pts, q_dotdot_pts, tau_ff_pts = self.generator.generate_chunk(
                self._next_chunk_start, chunk_end, q_start, self.frequency
            )

            # Convert time to oc.Time objects
            time_pts = oc.Time.convert_double_to_time_vector(t_vec)

            # Send the chunk
            # Note: send_dense_jointspace_trajectory expects q_dotdot and tau_ff
            # The tau_ff is added to the controller output on the real-time side
            self.robot.send_dense_jointspace_trajectory(
                time_pts, q_pts, q_dot_pts, q_dotdot_pts, tau_ff_pts
            )

            # Store last point for visualization
            if len(q_pts) > 0:
                self._last_sent_q = q_pts[-1]
                self._last_sent_q_dot = q_dot_pts[-1]
                self._last_sent_tau_ff = tau_ff_pts[-1]

            self._chunks_sent += 1
            self._points_sent += len(t_vec)
            # Estimate bytes: time(8) + 4*7joints*8bytes per point
            self._bytes_sent += len(t_vec) * (8 + 4 * 7 * 8)
            self._next_chunk_start = chunk_end
            return True

        except Exception as e:
            print(f"Error sending chunk: {e}")
            import traceback

            traceback.print_exc()
            return False

    def stop(self):
        """Stop the trajectory stream."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

        elapsed = (
            time.time() - self._stream_start_time if hasattr(self, "_stream_start_time") else 0
        )
        print("\nStreamer stopped.")
        print(f"  Chunks sent: {self._chunks_sent}")
        print(f"  Points sent: {self._points_sent}")
        print(f"  Data sent: {self._bytes_sent / 1e6:.2f} MB")
        if elapsed > 0:
            print(f"  Average bandwidth: {self._bytes_sent / elapsed / 1e6:.2f} MB/s")

    def is_running(self) -> bool:
        """Check if streamer is currently running."""
        return self._running

    def wait(self):
        """Wait for the trajectory stream to complete."""
        if self._thread:
            self._thread.join()

    def _stream_loop(self, q_start: np.ndarray, trajectory_end_time: float):
        """Main streaming loop running in background thread."""
        self._stream_start_time = time.time()
        last_log_time = 0
        last_viz_time = 0
        log_interval = 10.0  # Log progress every 10 seconds
        viz_interval = 0.1  # Update visualization every 100ms

        while self._running and self._next_chunk_start < trajectory_end_time:
            robot_time = self.robot.time.to_sec()
            buffer_time = self._next_chunk_start - robot_time

            # Keep buffer filled
            if buffer_time < self.max_buffer_duration:
                chunk_end = min(self._next_chunk_start + self.chunk_duration, trajectory_end_time)

                if not self._send_chunk(q_start, chunk_end):
                    self._running = False
                    break

                # Log progress periodically
                current_time = time.time()
                if current_time - last_log_time > log_interval:
                    elapsed = robot_time - self._trajectory_start_time
                    remaining = self._total_duration - elapsed
                    new_buffer = self._next_chunk_start - robot_time
                    progress = (elapsed / self._total_duration) * 100
                    print(
                        f"  Progress: {progress:.1f}% | Elapsed: {elapsed:.1f}s | "
                        f"Remaining: {remaining:.1f}s | Buffer: {new_buffer:.1f}s | "
                        f"Points sent: {self._points_sent}"
                    )
                    last_log_time = current_time

                # Warn if buffer is negative
                if buffer_time < 0:
                    print(f"  WARNING: Buffer negative ({buffer_time:.2f}s) - falling behind!")
            else:
                # Buffer is full, wait a bit
                time.sleep(0.05)

            # Update visualization periodically
            current_time = time.time()
            if self.visualization and current_time - last_viz_time > viz_interval:
                # Get trajectory target at CURRENT robot time (not the buffered future data)
                # This shows what the robot SHOULD be doing right now
                q_target, q_dot_target, tau_ff_target = self.generator.get_state_at_time(robot_time)

                self.visualization.add_sent_data(robot_time, q_target, q_dot_target, tau_ff_target)

                # Add actual robot state
                self.visualization.add_actual_data(
                    robot_time,
                    np.array(self.robot.state.q_act),
                    np.array(self.robot.state.q_dot_act),
                )

                # Add buffer info
                self.visualization.add_buffer_info(robot_time, buffer_time)

                last_viz_time = current_time

        self._running = False
        print("\nTrajectory stream complete!")
        print(f"  Total chunks sent: {self._chunks_sent}")
        print(f"  Total points sent: {self._points_sent}")
        print(f"  Total data sent: {self._bytes_sent / 1e6:.2f} MB")


# ============================================================================
# Main Example
# ============================================================================

if __name__ == "__main__":
    # =========================================================================
    # Configuration
    # =========================================================================

    # Set to False for real robot, True for simulation
    USE_SIMULATION = True

    # Robot IP configuration (only used if USE_SIMULATION = False)
    ROBOT_IP = "192.168.1.100"
    LOCAL_IP = "192.168.1.50"

    # Trajectory parameters. Override from env for smoke tests that need a
    # short run — e.g. ORC_DENSE_TRAJ_DURATION_MIN=0.1.
    DURATION_MINUTES = float(os.environ.get("ORC_DENSE_TRAJ_DURATION_MIN", "10.0"))
    DURATION = DURATION_MINUTES * 60.0  # Convert to seconds

    # Streaming parameters
    FREQUENCY = 1000  # 1kHz - upsampled to 8kHz on real-time side
    CHUNK_DURATION = 1.0  # 1 second chunks
    MAX_BUFFER = 30.0  # Buffer up to 30 seconds ahead

    # =========================================================================
    # Create robot instance
    # =========================================================================
    model_path = Path(__file__).parent.parent.parent / "models" / "iiwa_hanging.mjb"
    model_path = str(model_path)

    if USE_SIMULATION:
        robot = orco.Iiwa(model_path)
        print("Connecting to SIMULATION...")
    else:
        robot = orco.Iiwa(model_path=model_path, local_ip_addr=LOCAL_IP, robot_ip_addr=ROBOT_IP)
        print(f"Connecting to REAL ROBOT at {ROBOT_IP}...")

    # Wait for robot state
    print("Waiting for robot state...")
    time.sleep(0.5)

    if robot.time.to_sec() == 0:
        print("WARNING: Robot time is 0 - make sure simulate_iiwa.py is running!")
        time.sleep(1.0)

    if robot.time.to_sec() == 0:
        print("ERROR: No robot time received. Is simulate_iiwa.py running?")
        exit(1)

    print(f"Connected! Robot time: {robot.time.to_sec():.3f}s")
    print(f"Current joint config: {robot.state.q_act}")

    # =========================================================================
    # Select trajectory generator
    # =========================================================================

    # Looping trajectory: zeros -> ones -> zeros, 5 times per minute
    # tau_ff is kept at zero as requested
    generator = LoopingTrajectoryGenerator(
        dof=7,
        q_start=np.zeros(7),  # Start configuration
        q_end=np.ones(7),  # End configuration (1 radian on each joint)
        loops_per_minute=5.0,  # 5 complete cycles per minute = 12s per cycle
        duration=DURATION,
    )

    # Option 1: Simple sinusoidal trajectory (uncomment to use)
    # generator = SinusoidalTrajectoryGenerator(
    #     dof=7,
    #     amplitude=0.3,          # radians
    #     frequency_hz=0.05,      # 0.05 Hz = 20 second period
    #     joint_index=0,          # Move first joint
    #     duration=DURATION
    # )

    # Option 2: Load from NPZ file (uncomment to use)
    # Make sure the NPZ file contains 'q', 'qdot' (or 'q_dot'), and 'tau_ff' (or 'tau_ext')
    # generator = NPZTrajectoryGenerator(
    #     npz_path="/path/to/your/trajectory.npz",
    #     source_frequency=1000,  # Frequency of data in NPZ file
    #     dof=7
    # )
    # DURATION = generator.get_duration()

    # Option 3: Spline through waypoints with feedforward (uncomment to use)
    # q0 = np.zeros(7)
    # q1 = np.array([0.5, 0.3, -0.2, 0.4, -0.3, 0.2, 0.1])
    # q2 = np.array([-0.3, 0.5, 0.2, -0.4, 0.3, -0.2, -0.1])
    # q3 = np.zeros(7)
    # tau0 = np.zeros(7)  # Feedforward at waypoint 0
    # tau1 = np.array([1.0, 0.5, 0.3, 0.2, 0.1, 0.05, 0.02])  # Feedforward at waypoint 1
    # tau2 = -tau1  # Feedforward at waypoint 2
    # tau3 = np.zeros(7)  # Feedforward at waypoint 3
    # generator = SplineTrajectoryGenerator(
    #     waypoints=[q0, q1, q2, q3],
    #     durations=[120.0, 120.0, 120.0],  # 2 minutes between each
    #     dof=7,
    #     tau_ff_waypoints=[tau0, tau1, tau2, tau3]
    # )
    # DURATION = generator.get_duration()

    # =========================================================================
    # Move to initial position
    # =========================================================================
    q_initial = generator.get_initial_config()
    q_current = robot.state.q_act

    position_error = np.linalg.norm(q_initial - q_current)
    MOVE_THRESHOLD = 0.01  # radians

    if position_error > MOVE_THRESHOLD:
        print("\nMoving to initial trajectory position...")
        print(f"  Current: {q_current}")
        print(f"  Target:  {q_initial}")
        print(f"  Error:   {position_error:.4f} rad")

        move_duration = max(2.0, position_error * 2.0)

        robot.move_jointspace(
            q1=q_initial, t0=robot.time, T_traj=oc.Time(move_duration), N_pts=20, blocking_call=True
        )

        time.sleep(0.5)
        print(f"Reached initial position. Current: {robot.state.q_act}")
    else:
        print(f"Already at initial position (error: {position_error:.4f} rad)")

    ENABLE_VISUALIZATION = True  # Set to False to disable plots

    visualization = None
    if ENABLE_VISUALIZATION and MATPLOTLIB_AVAILABLE:
        print("\nStarting real-time visualization...")
        visualization = BufferVisualization(
            dof=7,
            window_seconds=15.0,  # Show last 15 seconds
            update_interval_ms=100,  # Update at 10Hz
            joints_to_plot=[0, 1, 2],  # Plot first 3 joints
        )
        visualization.start()
    elif ENABLE_VISUALIZATION:
        print("Visualization requested but matplotlib not available.")

    # =========================================================================
    # Start streaming trajectory
    # =========================================================================

    streamer = DenseTrajectoryStreamer(
        robot=robot,
        generator=generator,
        frequency=FREQUENCY,
        chunk_duration=CHUNK_DURATION,
        max_buffer_duration=MAX_BUFFER,
        visualization=visualization,
    )

    print(f"\nStreaming {DURATION / 60:.1f} minute trajectory at {FREQUENCY}Hz")
    print(f"Generator: {type(generator).__name__}")
    print("Feedforward torques (tau_ff) will be added to controller output")
    print(f"Real-time side upsamples from {FREQUENCY}Hz to 8kHz via linear interpolation")

    streamer.start(DURATION)

    try:
        # Wait for completion or Ctrl+C
        # If visualization is enabled, we need to keep the main thread alive for matplotlib
        if visualization:
            while streamer.is_running():
                plt.pause(0.1)  # This keeps matplotlib responsive
        else:
            streamer.wait()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        streamer.stop()
    finally:
        if visualization:
            visualization.stop()

    # Wait for final trajectory execution
    time.sleep(1.0)
    print("\nDone!")
