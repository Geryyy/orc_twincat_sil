Installation
================


---------------------------------------------
Installing orcpy via PyPI (not yet available)
---------------------------------------------

The `orcpy` Python package is planned to be published on PyPI, but that
distribution path is not yet implemented. For now, use the source-based
installation options below.




------------------------
Building from source
------------------------

The repository contains a devcontainer, which provides a convenient environment for building and developing ORC. In case you want to build ORC without the devcontainer, you can follow the instructions in the next sections.

Using the devcontainer
^^^^^^^^^^^^^^^^^^^^^^^^

The repository includes a devcontainer configuration in `.devcontainer/`.
With VS Code and the Dev Containers extension, open the repository and choose
**Dev Containers: Reopen in Container**. The container runs
`.devcontainer/postCreate.sh`, which installs the Python environment and prints
the available C++ and Python commands. The post-create step installs the editable
Python package with `pip install -e .`.

If you prefer the `Dev Containers CLI <https://containers.dev/supporting#devcontainer-cli>`_, you can also start it from the repository
root with:

.. code-block:: bash

    devcontainer up --workspace-folder .



Requirements
^^^^^^^^^^^^^^^^^

.. note::

    This is not required, if you are using the devcontainer, as it already has all dependencies installed.


ORC has the following dependencies that need to be installed prior to building the package:

.. code-block:: bash

    sudo apt install libeigen3-dev libboost-all-dev cmake

MuJoCo 3.3.2 has to be installed using CMake from source. A guide can be found `here`_.

.. _here: https://mujoco.readthedocs.io/en/stable/programming/index.html#building-from-source

Nanobind is installed during the CMake build process, so no manual installation is required.

Presuming you have met the requirements, you can build ORC and install
`orcpy` from the source tree by following these steps.

Building ORC from source:

.. code-block:: bash

    git clone https://github.com/anonymous/orc-a.git
    cd orc-a
    mkdir build && cd build
    cmake ..
    sudo make install


Installing orcpy (Python bindings)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


.. attention::

    We strongly suggest using a virtual environment for installing the Python bindings.

    .. code-block:: bash

        python -m venv .venv
        source .venv/bin/activate  # On Windows use `.venv\Scripts\activate`

Installing `orcpy` from source:

.. code-block:: bash

    git clone https://github.com/anonymous/orc-a.git
    cd orc-a
    pip install .

To install the documentation extras outside the devcontainer, run:

.. code-block:: bash

    pip install -e ".[doc]"

.. note::

    It is not necessary to build the C++ package before installing the Python bindings, as the required build steps are handled automatically during the pip installation process.

See also
--------

- :doc:`advanced/cmake`
