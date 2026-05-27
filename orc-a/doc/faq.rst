Frequently Asked Questions (FAQ)
======================================

Do I need any other packages for running orcpy?
-------------------------------------------------

Apart from the dependencies mentioned in the introduction, it is advisable to install `MujocoMeshManager <https://anonymous/mujocomeshmanager>`_ for handling larger robot models in simulation.

What models were used for the generation of the precompiled MJBs inside the models directory?
-----------------------------------------------------------------------------------------------------------

Some simple models, such as ``iiwa_standing.mjb``, ``iiwa_hanging.mjb`` were generated directly from their XML file, which can be found inside the ``models`` directory.

Larger models such as Pascal Bernoulli or other composite models containing multiple bodies have to be generated using the Python files inside ``models/generation_scripts``. The generated MJB files are saved inside ``models/presets`` and ignored by git. To use `MujocoMeshManager <https://anonymous/mujocomeshmanager>`_, install it in the same environment as ORC and make sure the dependencies listed in the introduction are available:

.. code-block:: bash

    pip install .

This pulls the MuJoCo version pinned by ORC so the mesh manager runs against the same simulator the controllers were tuned for.

I am having an issue installing MuJoCo 3.3.2 on my Windows PC?
-----------------------------------------------------------------

During development we ran into problems installing MuJoCo on some Windows 10 machines. This issue was reported `here <https://github.com/google-deepmind/mujoco/issues/2927>`_, and fixed in the most recent version of MuJoCo (as of November 2025). Since the fix is not implemented for prior version a working release with the implemented fix can be found `here <https://github.com/niko-mit-d/mujoco3.3.2-hotfix>`_. Building it this way will work assuming you're obeying the SDK requirements mentioned in the `MuJoCo documentation <https://mujoco.readthedocs.io/en/3.3.2/programming/#building-from-source>`_.

I am having an issue with orcpy, where can I get help?
------------------------------------------------------

If you encounter any issues while using orcpy, please feel free to open an issue on the `GitHub repository <https://github.com/anonymous/orc-a/issues/new>`_.
