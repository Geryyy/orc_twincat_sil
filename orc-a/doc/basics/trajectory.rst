``orc::trajectory`` submodule
==============================================

The trajectory module provides a complete trajectory abstraction for specifying robot motion over time. All trajectories inherit from a common base class and follow a consistent interface for initialization, update, and state management.

Base Class: ``TrajectoryBase<DOF>``
-----------------------------------

All trajectory types inherit from ``TrajectoryBase<DOF>``, providing:

- Initialization with or without saved state from a previous trajectory (for continuous hand-off).
- Time-based trajectory updates and state queries.
- Saving intermediate trajectory state for smooth transitions between successive trajectories.

State Hand-off class: ``TrajectoryPointStorage<DOF>``
-----------------------------------------------------

When trajectories are chained together (e.g., sending a new trajectory while the previous one is still executing), ``TrajectoryPointStorage`` is used to maintain continuity. It captures the desired position, velocity, and acceleration of the outgoing trajectory at the hand-off time.

The structure contains:

- ``Time time_`` — Absolute time of the hand-off point.
- ``JointVector q_, q_dot_, q_dotdot_`` — Desired joint configuration, velocity, and acceleration (for joint-space trajectories).
- ``PoseVector pose_`` — Desired end-effector pose (for task-space trajectories).
- ``CartesianVector x_dot_, x_dotdot_`` — Desired end-effector velocity and acceleration (for task-space trajectories).
- ``double force_`` — Desired force (for hybrid force-motion trajectories).

This data is passed to ``init(TrajectoryPointStorage saved_state)`` in the incoming trajectory so it can rebase its internal time basis and maintain smooth, continuous motion at the hand-off.

Trajectory Categories
---------------------

Trajectory types can be organized into two broad categories:

Executing Trajectories
^^^^^^^^^^^^^^^^^^^^^^^^

Move a robot from one state to another over a period of time, often involving interpolation between multiple waypoints.


.. raw:: html

   <details>
   <summary><strong>JointspaceTrajectory</strong></summary>

- **Purpose:** Joint-space point-to-point motion via smooth interpolation.
- **Interpolator:** ``SplineJointInterpolator``.
- **Controllers:**
  - Controller deriving from ``JointTrackingController``

.. raw:: html

   </details>

.. raw:: html

   <details>
   <summary><strong>TaskspaceTrajectory</strong></summary>

- **Purpose:** End-effector pose control in Cartesian space.
- **Interpolator:** ``CartesianPoseInterpolator``.
- **Controllers:**
  - Controller deriving from ``PoseTrackingController``

.. raw:: html

   </details>

.. raw:: html

   <details>
   <summary><strong>DenseJointspaceTrajectory</strong></summary>

- **Purpose:** Direct execution of pre-sampled offline trajectories with feedforward torques.
- **Interpolator:** None (pre-sampled lookup).
- **Controllers:**
  - Controller deriving from ``JointTrackingController``

.. raw:: html

   </details>

.. raw:: html

   <details>
   <summary><strong>HybridForceMotionTrajectory</strong></summary>

- **Purpose:** Simultaneous end-effector pose and contact force tracking.
- **Interpolator:** ``CartesianPoseInterpolator`` + ``SplineJointInterpolator<1>`` for force.
- **Controllers:**
  - ``HybridForceMotionController``

.. raw:: html

   </details>

.. raw:: html

   <details>
   <summary><strong>CartesianVelocityTrajectory</strong></summary>

- **Purpose:** Velocity-based end-effector control (no explicit orientation tracking).
- **Interpolator:** ``SplineJointInterpolator<6>``.
- **Controllers:**
  - Controller deriving from ``PoseTrackingController``

.. raw:: html

   </details>

.. raw:: html

   <details>
   <summary><strong>JointspaceVelocityTrajectory</strong></summary>

- **Purpose:** Velocity-based joint-space motion with smooth acceleration.
- **Interpolator:** ``SplineJointInterpolator``.
- **Controllers:**
  - Controller deriving from ``JointTrackingController``

.. raw:: html

   </details>

Single-Event Trajectories
^^^^^^^^^^^^^^^^^^^^^^^^^

Single-event trajectories do not evolve over time; they represent instantaneous changes to control parameters or robot state.

.. raw:: html

   <details>
   <summary><strong>JointCtrParamTrajectory</strong></summary>

- **Purpose:** Change ``JointCTController`` parameters (gains K0, K1, KI) mid-execution.
- **Interpolator:** None.
- **Controllers:**
  - ``JointCTController``

.. raw:: html

   </details>

.. raw:: html

   <details>
   <summary><strong>CartesianCtrParamTrajectory</strong></summary>

- **Purpose:** Change ``CartesianCTController`` parameters mid-execution.
- **Interpolator:** None.
- **Controllers:**
  - ``CartesianCTController``

.. raw:: html

   </details>

.. raw:: html

   <details>
   <summary><strong>NullspaceTrajectory</strong></summary>

- **Purpose:** Specify desired nullspace joint configuration without affecting end-effector task.
- **Interpolator:** None.
- **Controllers:**
  - ``CartesianCTController``

.. raw:: html

   </details>
