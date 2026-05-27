``orc::interpolator`` submodule
==============================================

Overview
--------

Interpolators generate smooth setpoints between waypoints. They provide the time-parameterized motion that trajectories consume to produce desired joint positions, task-space poses, velocities, and accelerations.

ORC implements its interpolators on top of Eigen's `Spline and spline fitting module <https://libeigen.gitlab.io/eigen/docs-nightly/unsupported/group__Splines__Module.html>`_. This gives the library a common numerical backend for fitting and evaluating smooth splines in both joint space and Cartesian space.

How interpolators work with trajectories
----------------------------------------

Interpolators are not used directly by the robot control loop. Instead, they are embedded inside trajectory classes such as ``JointspaceTrajectory`` and ``TaskspaceTrajectory``.

A typical flow is:

1. A trajectory stores its waypoint data and time points.
2. The trajectory constructs the matching interpolator.
3. During ``update(Time t)``, the trajectory asks the interpolator for the setpoint at the requested time.
4. The interpolated point, velocity, and acceleration are copied into the trajectory state or ``RobotData``.
5. The controller then consumes the updated trajectory state.

This separation keeps interpolation logic independent from controller logic, and allows different trajectory types to reuse the same interpolation backend.

Built-in interpolators
----------------------

``SplineJointInterpolator``
^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Smooth spline interpolation for joint-space data.
- Used by joint-space trajectories and related velocity trajectories.
- Fits a spline through joint-space waypoints and evaluates position, velocity, and acceleration.

``CartesianPoseInterpolator``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Smooth interpolation for Cartesian pose trajectories.
- Used by task-space trajectories.
- Interpolates position and orientation together so that trajectories can produce continuous end-effector poses.

Implementing a custom interpolator
----------------------------------

If you want to add a new interpolator, the typical steps are:

1. Choose the appropriate base class:
   - ``ManifoldInterpolatorBase`` for interpolators that work on manifold-valued states such as poses.
   - ``InterpolatorBase`` or the existing concrete pattern used by the current joint-space interpolators for simple Euclidean state types.
2. Define the input and output types for your interpolation problem.
3. Store the waypoint/time data in the interpolator object.
4. Implement initialization and fitting logic for the spline or interpolation scheme you need.
5. Implement evaluation so the interpolator can return the interpolated state and its derivatives at a given time.
6. Add the new interpolator header to ``include/orc/interpolator/`` and include it from ``include/orc/interpolator/Interpolator.h`` if it should be part of the common umbrella header.
7. Use the new interpolator inside a trajectory class so it can be exercised through the existing controller/trajectory flow.
8. Add a trajectory or interpolator test that checks continuity, boundary behavior, and derivatives at the endpoints.

See also
--------

- :doc:`trajectory`
- :doc:`controller`
- :doc:`../architecture`
