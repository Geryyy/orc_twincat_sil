.. _control_submodule:

``orc::control`` submodule
==============================================

The control module provides the controller hierarchy used by ORC to generate joint torques and velocities from the current robot state. Controllers are split into primary tracking controllers and secondary controllers that can be combined with them.

Abstract controller hierarchy
-----------------------------

``ControllerBase`` is the common abstract base class for all controllers in ORC. It stores read-only access to ``orc::robots::RobotData`` and requires each controller to implement:

- ``update()`` to compute the controller output from the current robot state.
- ``reset()`` to reinitialize controller internals when a trajectory or control mode changes.

``JointTrackingController`` and ``PoseTrackingController`` are the two primary abstract specializations of ``ControllerBase``:

- ``JointTrackingController`` is the base for controllers that track joint-space trajectories.
- ``PoseTrackingController`` is the base for controllers that track task-space or pose-based trajectories.

Both classes add the semantic distinction between joint-space and pose-space tracking, while keeping the same abstract controller interface.

Controller composition
----------------------

ORC allows one primary controller and multiple secondary controllers to be active at the same time. The primary controller produces the main commanded torque, while secondary controllers add additional torque contributions for auxiliary objectives such as gravity compensation or friction compensation.

.. math::
   :label: eq:torque

   \vec{\tau} = \vec{\tau}_\text{primary} + \sum_i\vec{\tau}_{\text{secondary},i}

In practice, this composition is set up through the ``register_*`` controller functions in :cpp:class:`orc::robots::Robot`. The robot stores the primary tracking controller separately and keeps the secondary controllers in a dedicated list, so their torques are accumulated during the update step.


Controller inventory
--------------------

Pose-tracking controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^

These controllers track task-space motion and are used for Cartesian or pose-based trajectories.

.. raw:: html

   <details>
   <summary><strong>CartesianCTController</strong></summary>

- Primary pose-tracking computed torque controller for task-space trajectories.
- Registered via ``register_CartesianCTController``.

.. raw:: html

   </details>

.. raw:: html

   <details>
   <summary><strong>HybridForceMotionController</strong></summary>

- Primary pose-and-force controller for hybrid force-motion trajectories.
- Registered via ``register_HybridForceMotionController``.

.. raw:: html

   </details>

Joint-tracking controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^

These controllers track joint-space motion and are used for joint-space trajectories or joint-velocity commands.

.. raw:: html

   <details>
   <summary><strong>JointCTController</strong></summary>

- Primary computed-torque controller for joint-space trajectories.
- Registered via ``register_JointCTController``.

.. raw:: html

   </details>

.. raw:: html

   <details>
   <summary><strong>JointPDPController</strong></summary>

- Joint-space PD+ controller.
- Registered via ``register_JointPDPController``.

.. raw:: html

   </details>

.. raw:: html

   <details>
   <summary><strong>VelocityController</strong></summary>

- Joint-space velocity controller. The output of the controller is the set velocity.
- Registered via ``register_VelocityController``.

.. raw:: html

   </details>

Secondary controllers
^^^^^^^^^^^^^^^^^^^^^

These controllers are added on top of the primary controller to provide additional objectives or compensation.

.. raw:: html

   <details>
   <summary><strong>GravityCompController</strong></summary>

- Gravity compensation with damping.
- Registered via ``register_GravityCompController``.

.. raw:: html

   </details>

.. raw:: html

   <details>
   <summary><strong>SingularPerturbationController</strong></summary>

- Secondary controller for singular perturbation-based compensation.
- Registered via ``register_SingularPertrubationController``.

.. raw:: html

   </details>

.. raw:: html

   <details>
   <summary><strong>FrictionCompController</strong></summary>

- Friction compensation controller.
- Registered via ``register_FrictionCompController``.

.. raw:: html

   </details>

.. raw:: html

   <details>
   <summary><strong>CoulombFrictionCompController</strong></summary>

- Coulomb and viscous friction compensation controller.
- Registered via ``register_CoulombFrictionCompController``.

.. raw:: html

   </details>

Writing a custom controller
---------------------------

If you want to add a controller that is not part of the built-in set, the usual pattern is:

1. Choose the right base class:
   - ``ControllerBase<DOF>`` for fully custom behavior.
   - ``JointTrackingController<DOF>`` for joint-space tracking controllers.
   - ``PoseTrackingController<DOF>`` for task-space or pose-tracking controllers.
2. Add a parameter struct for the controller gains and configuration values.
3. Implement ``update()`` to compute the control output from ``RobotData``.
4. Implement ``reset()`` so the controller can be reinitialized when trajectories or operating modes change.
5. Register the controller inside ``orc::robots::Robot`` using a dedicated ``register_<Name>Controller()`` helper, following the same pattern as the built-in controllers.
6. If the controller should act as a secondary controller, store it in the robot's secondary-controller list so its output is added to the primary command during each control cycle.

A minimal custom controller therefore has three pieces: a base class, a parameter bundle, and a robot registration function. The robot class then takes care of calling the controller from ``Robot::update()`` and resetting it from ``Robot::reset()`` when needed.

Robot data access
-----------------

Controllers communicate with the robot through ``orc::robots::RobotData``, which contains the current and desired joint states, torques, mass matrix, Jacobians, and end-effector pose information. Controllers only receive read-only access to this structure, which keeps the control loop data flow explicit and avoids accidental mutation of robot state.

This is described further in the :ref:`orc::robots submodule documentation <robots_submodule>`.
