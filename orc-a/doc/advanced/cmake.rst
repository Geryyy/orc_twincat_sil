CMake build system
===================

ORC uses CMake as its main build system. The top-level ``CMakeLists.txt`` configures the core C++ library, optional examples, tests, documentation, FlatBuffers header generation, and the Python bindings.

The build is intentionally option-driven: the most common knobs are exposed as CMake cache options, while dependency discovery is handled through ``find_package`` calls for Boost, MuJoCo, and Eigen.

Basic configuration
-------------------

A typical out-of-source configure/build sequence looks like this:

.. code-block:: bash

   cmake -S . -B build
   cmake --build build -j

Useful configuration variables include:

- ``CMAKE_BUILD_TYPE`` — Controls the build type for single-config generators such as Makefiles or Ninja.
  - The top-level project defaults to ``Debug`` when no build type is selected.
- ``CMAKE_CXX_COMPILER`` — Selects the C++ compiler.
  - If no compiler is set, ORC tries to use ``g++-12`` when it is available.
- ``CMAKE_CXX_STANDARD`` — ORC requires C++20.
- ``CMAKE_EXPORT_COMPILE_COMMANDS`` — Enabled by default to generate ``compile_commands.json`` for tooling.

Required dependencies
---------------------

The top-level build requires:

- Boost ``1.71`` through ``1.83``,
- MuJoCo ``3.3.2``,
- Eigen ``3.4`` or newer.

If configuration fails, check the dependency versions first, because these requirements are enforced during the configure step.

ORC-specific build options
--------------------------

These are the most relevant ORC build options exposed by the top-level project:

``BUILD_FLATBUFFERS``
^^^^^^^^^^^^^^^^^^^^^

- Default: ``OFF``
- When enabled, CMake regenerates the checked-in FlatBuffers C++ header from ``proto/orc_messages.fbs`` using ``flatc``.
- Use this when the FlatBuffers schema changes.

``BUILD_EXAMPLES``
^^^^^^^^^^^^^^^^^^

- Default: ``OFF``
- Builds the example executables from ``examples/``.
- Useful when you want to run or extend the shipped demo programs.

``BUILD_TESTS``
^^^^^^^^^^^^^^^

- Default: ``OFF``
- Builds the unit and regression tests in ``tests/`` and enables CTest integration.

``CODE_COVERAGE``
^^^^^^^^^^^^^^^^^

- Default: ``OFF``
- Adds coverage compiler/linker flags for GCC and Clang.
- Intended for local validation and CI coverage runs.

``BUILD_DOXYGEN``
^^^^^^^^^^^^^^^^^

- Default: ``OFF``
- Builds the Doxygen documentation targets from ``doc/``.
- The Sphinx documentation is built separately under ``doc/``.

Python bindings and ``SKBUILD``
-------------------------------

The Python bindings live under ``python/`` and are built when the ``SKBUILD`` variable is present.
This is the path used by ``pip install .`` and by scikit-build-based packaging.

In practice this means:

- ``SKBUILD`` is set automatically during the Python package build.
- The Python CMake project then builds the ``orcpy`` extension module via nanobind.
- The bindings are linked against the main ``orc`` library.

Common build recipes
--------------------

Build the C++ library plus tests and examples:

.. code-block:: bash

   cmake -S . -B build -DBUILD_TESTS=ON -DBUILD_EXAMPLES=ON
   cmake --build build -j

Enable FlatBuffers regeneration:

.. code-block:: bash

   cmake -S . -B build -DBUILD_FLATBUFFERS=ON
   cmake --build build -j

Build coverage-enabled binaries:

.. code-block:: bash

   cmake -S . -B build -DBUILD_TESTS=ON -DCODE_COVERAGE=ON
   cmake --build build -j

Build the Python package from the source tree:

.. code-block:: bash

   pip install .

See also
--------

- :doc:`../install`
