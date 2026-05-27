orc - OpenRobotControl
====================================================

**OpenRobotControl (orc)** is an open-source C++20 library for robot manipulation control that runs identical user-level code in simulation and on hard real-time deployment targets. Its central design principle is deployment parity: switching between simulation and physical hardware requires only an IP address change.

A lightweight client-server architecture decouples trajectory planning (Python or C++) from deterministic execution on the real-time target. All motion is expressed as typed trajectory objects pushed onto a queue; the server reconstructs, interpolates, and tracks them with model-based controllers each cycle. Trajectories, interpolators, and controllers are defined via minimal abstract interfaces, allowing users to extend the library without modifying it. orc provides built-in trajectory types, controller variants including computed-torque and hybrid force-motion control, Python bindings through `nanobind`_, and predefined robot platforms.

Kinematic calculations are using the `MuJoCo physics engine`_ at their core. MuJoCo allows for simple robot definitions using their `MJCF modelling language`_, which directly integrates with ORC.

.. _nanobind: https://nanobind.readthedocs.io/en/latest/index.html#
.. _MuJoCo physics engine: https://mujoco.readthedocs.io/en/stable/overview.html
.. _MJCF modelling language: https://mujoco.readthedocs.io/en/stable/XMLreference.html

---------------------
Key features
---------------------

ORC's key features include:
   - **Modular architecture** for easy extension and customization.
   - **Support for various robots**: Out of the box, ORC can be used on the KUKA LBR iiwa and Kinova Gen 3 robot.
   - **Robust trajectory queue** for managing and executing multiple trajectories seamlessly.
   - **Real-time control** capabilities for both simulation and real hardware.
   - **Python bindings** for easy scripting and rapid prototyping.
   - **Comprehensive documentation** and examples to help users get started quickly.


------------------------------------------
Dependencies
------------------------------------------
ORC depends on
   - **C++20** or higher
   - **CMake 3.15+**
   - **Eigen** library
   - **MuJoCo 3.3.2**
   - **Boost** version 1.71 to 1.83
   - **Python 3.10+**
   - **nanobind 2.9+** for Python bindings
   - **FlatBuffers 2.0+** for communication serialization

------------------------------------------
Citing this project
------------------------------------------
.. code-block:: bibtex

    @misc{orc,
       author = {anonymous},
       year = {2026},
       note = {},
       title = {OpenRobotControl (orc): Unified Simulation and Real-Time Control for Manipulators}
    }


.. only:: not latex

------------------------------------------
Table of contents
------------------------------------------

.. toctree::
   :maxdepth: 1

   introduction
   install
   getting_started
   architecture
   conventions
   faq
   changelog

.. toctree::
   :maxdepth: 1
   :caption: Basic components

   basics/trajectory
   basics/controller
   basics/interpolator
   basics/com
   basics/robots

.. toctree::
   :maxdepth: 1
   :caption: Advanced Usage

   advanced/cmake

.. toctree::
   :maxdepth: 1
   :caption: API Reference

   api/orcpy
   api/orc
