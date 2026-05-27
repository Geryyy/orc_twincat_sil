Introduction
=================

ORC's main goal is to simplify and streamline the process of robot control, from trajectory planning to execution. Its modular architecture allows users to easily adapt and extend the framework to suit their specific needs.

--------------------------------
Data flow
--------------------------------

At the heart of ORC lies the ``orc::robots::Robot`` class. The class is used to pass measurement data and to plan and execute trajectories using previously defined controllers. ``Robot`` contains an ``update`` function, which based on passed measurements and active (or inactive) trajectories, computes control commands.

``Robot`` consists of several components:

- MuJoCo model of robot for model-based calculations.
- TrajectoryQueue managing active and planned trajectories, as well as trajectory serializers and deserializers for communication over UDP.
- Controllers computing control commands based on measurements and trajectory type.

The user is able to obtain or pass information to a ``Robot`` instance using:

- ``set_X`` and ``get_X`` functions for various data types (e.g. joint positions, velocities, torques, end-effector poses, etc.).
- ``register_XController`` functions to add controllers to the robot.
- As well as ``add_trajectory`` and ``add_x_trajectory`` functions to add trajectories to the trajectory queue.

.. note::
    The user should be aware of the measurements necessary for the controllers in use. Make sure to read the :ref:`specific controller documentation <control_submodule>` for more information.


See :doc:`architecture` for a module dependency diagram, timing model,
and real-time expectations.

----------------------------------------------------------------
Predefined classes deriving from ``orc::robots::Robot``
----------------------------------------------------------------


For maximum flexibility the ``Robot`` class can be used directly. This creates a blank slate, where the user has to manually define controllers. However, ORC also provides predefined robot classes that come with commonly used controllers and trajectory types already implemented, e.g.:

- ``orc::robots::Iiwa``: KUKA LBR iiwa robot with joint and cartesian computed torque controllers running. Additionally secondary controllers, i.e., friction compensation, and a singular perturbation controller are used for better performance. A gravity compensation controller is also available out of the box and can be triggered using the ``grav_comp_only`` argument of the update function.
- ``orc::robots::Kinova``: Kinova Gen3 (7DOF) robot with the same primary/secondary controller stack as the Iiwa wrapper and Kinova-specific default parameters and home configuration.
- ``orc::robots::LinearAxis``: A 2DOF linear axis robot with a velocity controller running.
- ``orc::robots::DummyRobot``: A controller-less placeholder used by the unit tests; useful as a starting point for writing your own ``Robot`` subclass.

The Python package additionally ships ``orcpy.robots.Robot9DOF``, a composite wrapper that stacks an :class:`Iiwa` on top of a :class:`LinearAxis` for combined 9-DOF setups.

----------------------------------------------------------------
Components
----------------------------------------------------------------

ORC's submodules are described in the `Basic components` section.
