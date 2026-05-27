.. _robots_submodule:

``orc::robots`` submodule
==============================================


``orc::robots::Robot`` is the runtime integration point of ORC. It combines the communication layer, trajectories, interpolators, controllers, and the robot model into one control loop.

At a high level, ``Robot`` is responsible for:

- loading and owning the MuJoCo model and data,
- maintaining ``orc::robots::RobotData`` for the active controllers,
- receiving, storing, and dispatching trajectories,
- serializing and deserializing communication payloads,
- registering and invoking the active controllers,
- running the control-cycle update function.

How the submodules fit together
-------------------------------

The preceding submodules are used by ``Robot`` in the following way:

- :doc:`com` provides the FlatBuffers-based communication layer used to send and receive trajectory messages.
- :doc:`trajectory` provides the trajectory objects that are stored in the queue and executed over time.
- :doc:`interpolator` provides the spline-based interpolation backends used inside the trajectory classes.
- :doc:`controller` provides the primary tracking controllers and secondary controllers that compute the commanded torques or velocities.

During a control cycle, ``Robot`` updates ``RobotData``, advances the active trajectory, evaluates the registered controllers, and combines the resulting control actions into the final command.

Robot data flow
---------------

The update loop is centered around ``orc::robots::RobotData``. The robot class collects the current measurements, computes the derived model quantities, and exposes the resulting data to the controllers as read-only input.

Typical flow:

1. Measurements are provided to the robot through the appropriate ``set_*`` functions.
2. ``RobotData`` is updated with the current state, model terms, and derived quantities.
3. The active trajectory produces the desired setpoints.
4. The primary controller computes the main control action.
5. Secondary controllers contribute additional torque terms.
6. The final command is sent back through the robot interface.

Controller handling and composition
-----------------------------------

The robot class does not hard-code a single controller setup. Instead, users register the controllers they need for a given robot model.

The main controller categories are:

- ``JointTrackingController`` for joint-space trajectories,
- ``PoseTrackingController`` for task-space trajectories,
- secondary controllers for auxiliary objectives such as gravity compensation or friction compensation.

Only one primary tracking controller of each relevant type is typically registered, while multiple secondary controllers can be registered and combined. Their contributions are added to the primary control action during each update cycle.

This is why concrete robot classes usually call the registration functions during construction. The specialized robot classes in ORC are built exactly this way.

Custom robot classes
--------------------

If your robot model should be easier to use than the generic ``Robot`` class, add a small subclass that configures the robot for a specific mechanism.

A custom robot class typically does the following:

1. Inherit from ``orc::robots::Robot<DOF>``.
2. Define aliases for the robot-specific controller and trajectory types.
3. Provide a constructor that loads the model and registers the default controllers.
4. Set the robot-specific end-effector site / sensor names.
5. Add convenience methods such as a home configuration, startup sequence, or robot-specific parameter bundle.
6. Reuse the base class ``start()``, ``update()``, trajectory queue, and communication helpers.

The existing subclasses ``Iiwa``, ``LinearAxis``, and ``Kinova`` are good references:

- ``Iiwa`` wires together joint-space, Cartesian, secondary, and hybrid-force controllers.
- ``LinearAxis`` focuses on a simpler velocity-controlled setup.
- ``Kinova`` shows a compact robot-specific wrapper with default parameters and a home configuration.

MuJoCo model and data
---------------------

The MuJoCo model and data are owned internally by ``Robot``. The model is loaded from a provided MJB file or binary during construction, and the resulting data structure is used for all model-based control computations.

MuJoCo is the working horse for kinematics, dynamics, and derived quantities such as mass matrices and Jacobians.

Trajectory queue and communication
-----------------------------------

The trajectory queue stores incoming trajectory objects and exposes the currently active one to the control loop. Incoming messages are serialized and deserialized through the communication layer described in :doc:`com`.

This keeps the control loop decoupled from the transport format while still allowing trajectories to be streamed over UDP or handled locally.

Control cycle update function
-----------------------------

The update function is the heart of ``orc::robots::Robot``. It is called at each control cycle and is responsible for updating ``RobotData``, advancing the active trajectory, invoking the registered controllers, and computing the control command.

If the calculation succeeds, the function returns ``true``; otherwise it returns ``false`` and reports the issue through the logging system.

.. warning::

    Before calling ``update()``, the user must provide the measurements required by the registered controllers using the appropriate ``set_*`` functions. For example, a joint-tracking controller needs joint position and velocity measurements, while friction compensation controllers also require torque measurements. See :ref:`control_submodule` for the controller-specific requirements. Failing to provide the required data can lead to undefined behavior.
