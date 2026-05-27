Changelog
======================================

Unreleased
--------------------------------------

**Repository and build tooling**:

- Added a devcontainer for local development, plus updated CMake, packaging, CI, release, and documentation build workflows.
- Added project metadata and contributor-facing files such as `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`, `SECURITY.md`, `CITATION.cff`, and repository templates.
- Added formatting, linting, and pre-commit configuration for the codebase.

**Documentation and examples**:

- Reworked the documentation set with a new architecture overview, conventions, installation guidance, and expanded getting-started content.
- Split examples into C++ and Python groups and added many new runnable Python examples and paper-related experiment workflows.
- Added documentation and scripts for generating Kinova lemniscate figures and related experimental data.

**Core ORC, control, and trajectories**:

- Added FlatBuffers-based communication for robot state and trajectories, including new serializers, wire-format helpers, and generated schema code.
- Expanded controller and trajectory support with hybrid force-motion, Coulomb friction compensation, dense jointspace trajectories, velocity trajectories, and broader trajectory queue handling.
- Improved robot, interpolator, spline, time, filter, and logging internals for correctness, RT compatibility, and TwinCAT support.

**Python bindings and `orcpy`**:

- Expanded `orcpy` bindings for the updated controller, robot, trajectory, and communication APIs.
- Added Kinova Python communication support, trajectory sender helpers, type hints, and Python package metadata updates.
- Added Python-side example updates, constructor parity tests, attribute-name checks, and dynamic/static binding coverage.

**Robot models and integrations**:

- Added or updated Kinova, Iiwa, LinearAxis, and Robot9DOF integrations, including Kinova simulation, calibration, and lemniscate workflows.
- Added generation scripts and preset model binaries for several robot setups, including new MuJoCo model assets.
- Added FlatBuffers and Kortex third-party assets required by the new communication and robot workflows.

**Testing and validation**:

- Added broad C++ and Python regression coverage for serialization, trajectories, interpolators, robot behavior, and example execution.
- Added data fixtures and helper scripts for trajectory, plotting, and paper-experiment validation.

v0.2.0
--------------------------------------

**Hybrid force motion capability**:

- Added support for hybrid force motion control with ``control::HybridForceMotionController``, as well as own trajectory type ``trajectory::HybridForceMotionTrajectory``.
- Added examples under ``examples/python/hybrid_force_motion``.

**Kinova implementation**:

- Added working and useable Kinova Gen3 implementation in `orc-kinova-gen3-kortex repository <https://github.com/niko-mit-d/orc-kinova-gen3-kortex>`_.

**Breaking changes**:

- Added a virtual reset function to ControllerBase. This way every controller needs to implement a reset function.
- Added ControllerType enum: Since TwinCAT doesn't allow for dynamic down-casting this enum is added to every controller inheriting from ControllerBase.
- Changed all calculations regarding end-effector coordinate system to be with respect to a end-effector site. Previously a MuJoCo body was used. This improves flexibility. XMLs and MJBs have also been changed also due to this change.
- Added ``force_`` property to ``trajectory::TrajectoryPointStorage`` for before mentioned new trajectory type.


**orcpy**:

- Added pose vector support to ``Robot.move_taskspace()``.
- Added generation scripts for Pascal Bernoulli and Pascal with pen attached setup in the ``models/generation_scripts`` directory. With these scripts the model binaries can easily be generated. Generated MJBs are saved to ``models/presets``. As these MJBs can become large in size, they are ignored by git.
- Changed location of ``orcpy.robots.Robot9DOF`` default configuration.
- Bindings for all non ``robots::Robot`` instance-specific classes like ``RobotData``, interpolators, etc. are moved to ``orcpy.robotX`` namespace with X being equal the DoF.

**Minor fixes**:

- Fixed wrong maximum data size for UPD package, see `#22 <https://github.com/niko-mit-d/orc-a/issues/22>`_.
- Added CoulombFrictionCompController to ``orc::control``. It is currently not used in any pre-implemented robot classes.
- Added tests.
- Updated examples and added experiments for ORC paper.
- Added inverse kinematics functions to ``util_functions.py``.

v0.1.2
--------------------------------------

Same changes as v0.2.0. Pushed previously for testing.
