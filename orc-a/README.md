# ORC - OpenRobotControl

**OpenRobotControl** (**orc**) is an open-source C++20 library for robot manipulation control that runs identical user-level code in simulation and on hard real-time deployment targets.
Its central design principle is deployment parity: switching between simulation and physical hardware requires only an IP address change.
A lightweight client-server architecture decouples trajectory planning (Python or C++) from deterministic execution on the real-time target.
All motion is expressed as typed trajectory objects pushed onto a queue; the server reconstructs, interpolates, and tracks them with model-based controllers each cycle.
Trajectories, interpolators, and controllers are defined via minimal abstract interfaces, allowing users to extend the library without modifying it.
orc provides built-in trajectory types, controller variants including computed-torque and hybrid force-motion control, Python bindings, and predefined robot platforms.

Experimental validation on a Kinova Gen3 (1 kHz, Ubuntu), a KUKA LBR iiwa (8 kHz, TwinCAT), and an 18-DoF dual-arm system demonstrates simulation-hardware parity, stable force control, and multi-robot coordination.

## Getting started

The recommended setup for this submission is the included VS Code devcontainer:

1. Open this repository in VS Code.
2. Install the Dev Containers extension if needed.
3. Run **Dev Containers: Reopen in Container**.

After the container is built, `.devcontainer/postCreate.sh` runs `pip install -e .`
so the editable `orcpy` Python package is available in the container.

> **Wait for postCreate to finish before running examples.** Compiling the C++
> bindings takes ~30 s. Watch the **Dev Containers** output panel for the
> `postCreateCommand finished` message; opening a terminal earlier will hit
> `ModuleNotFoundError: No module named 'orcpy'`.

Run a quick simulation example with:

```bash
python examples/python/simulate_kinova.py
```

> **Linux GUI from container.** MuJoCo viewer needs host X server access.
> Run once on host before launching the example:
>
> ```bash
> xhost +local:docker
> ```

Local documentation sources are in [`doc/`](doc/), with installation details in
[`doc/install.rst`](doc/install.rst) and a quick start in
[`doc/getting_started.rst`](doc/getting_started.rst).

## Citation

If you use orc in your research, please cite the following paper:

```latex
@misc{anonymous,
       author = {anonymous},
       year = {2026},
       title = {OpenRobotControl (orc): Unified Simulation and Real-Time Control for Manipulators}
    }
```

## License

orc is distributed under the BSD-3-Clause. See [`LICENSE`](LICENSE).
