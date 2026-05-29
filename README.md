# orc-twincat-sil

A self-contained demo of [**ORC** (OpenRobotControl)](orc-a/README.md) running
on **TwinCAT** in hard real-time, with the robot **simulated in the loop (SIL)**
so you can try it without any physical hardware.

The ORC controller and a MuJoCo simulation of a KUKA LBR iiwa run **together on
a TwinCAT target at 8 kHz**. A Python client (typically in the ORC devcontainer)
streams trajectories to the target and visualizes the returned robot state as a
digital twin:

```
┌──────────────────────────┐         UDP          ┌────────────────────────────────────────────┐
│  Client (devcontainer)   │  trajectory  ─────▶  │  TwinCAT target (hard real-time, 8 kHz)      │
│  orcpy / Python          │   :10000             │                                              │
│  client_with_            │                      │   UdpInterface  ──▶  OrcController (ORC)      │
│  visualization.py        │  RobotState  ◀─────  │        ▲                    │ torque         │
│  (digital-twin viewer)   │   :11000             │        └── RobotSimulation ◀┘ (MuJoCo)       │
└──────────────────────────┘                      └────────────────────────────────────────────┘
```

Because ORC keeps identical user-level code in simulation and on hardware,
this same project becomes a real-robot deployment by swapping the simulation
module for EtherCAT motor drives — see the migration notes in the
[IiwaControl README](IiwaControl/README.md#migrating-to-a-real-robot).

## Repository layout

| Path | What it is |
| --- | --- |
| [`IiwaControl/`](IiwaControl/) | The TwinCAT solution (controller + simulation + UDP interface). **Start here** — see its [README](IiwaControl/README.md) for full setup and run instructions. |
| [`orc-a/`](orc-a/) | The ORC library (git submodule): C++ control library, `orcpy` Python bindings, and the [Python client examples](orc-a/examples/python/README.md). |
| [`mujoco_tc/`](mujoco_tc/) | Real-time-safe port of the MuJoCo physics engine, used by the in-TwinCAT robot simulation. |
| [`mujoco_models/`](mujoco_models/) | Precompiled MuJoCo models (`.mjb`) for the iiwa and linear axis (see [model constraints](#mujoco-model-constraints)). |
| [`TcIntrin/`](TcIntrin/) | TwinCAT math intrinsics headers (per toolset: v140/v141/v142). |
| [`eigen_import_libs/`](eigen_import_libs/) | Eigen compatibility shims for the TwinCAT C++ toolchain. |
| [`img/`](img/) | Screenshots used by the documentation. |

## TwinCAT solution modules

The real-time control loop in [`IiwaControl/`](IiwaControl/) is built from three
C++ modules running in the same 125 µs (8 kHz) task:

| Module | Role |
| --- | --- |
| `UdpInterface` | UDP endpoint — receives trajectories, sends back `RobotState`. |
| `OrcController` | The ORC computed-torque controller; produces joint torques. |
| `RobotSimulation` | MuJoCo simulation of the iiwa; integrates torques into joint state (replace with EtherCAT drives for a real robot). |

## Getting started

1. **Set up and build the TwinCAT project.** Follow the
   [IiwaControl README](IiwaControl/README.md) — it covers TwinCAT/C++
   installation, the C++ project configuration, network/IP settings, building,
   and activating the configuration.
2. **Run the Python client.** From the ORC devcontainer, stream a trajectory
   and watch the digital twin:
   ```bash
   python examples/python/client_with_visualization.py \
       --robot-ip <twincat-target-ip> --local-ip <client-ip>
   ```
   See [the run section](IiwaControl/README.md#5-run-the-python-client-with-visualization)
   for IP details, and the [ORC README](orc-a/README.md) for the devcontainer.

## MuJoCo model constraints

The in-TwinCAT simulation loads a precompiled MuJoCo binary (`.mjb`). To be
real-time-loadable a model must:

- have exactly 7 joints (`model.nq == 7`);
- be built with **MuJoCo 3.3.2** via
  [`mj_saveModel`](https://mujoco.readthedocs.io/en/3.3.2/APIreference/APIfunctions.html#mj-savemodel);
- contain no meshes — set
  [`discardvisual`](https://mujoco.readthedocs.io/en/3.3.2/XMLreference.html#compiler-discardvisual)
  to `true`;
- avoid exotic features (deformable objects, height fields, etc.), which are
  not supported.

Copy the chosen `.mjb` into `C:\TwinCAT\3.1\Boot\` so the real-time module can
load it.

## License

ORC is distributed under the BSD-3-Clause license; see
[`orc-a/LICENSE`](orc-a/LICENSE).
