# Python Examples

Short, self-contained scripts demonstrating `orcpy` usage. All examples
resolve bundled model files via
[`orcpy.robots.util_functions.default_model_path`](../../python/robots/util_functions.py),
so they can be executed from any working directory (override with the
`ORCPY_MODELS_DIR` environment variable).

| Script | Purpose | Required model(s) | Example command |
| --- | --- | --- | --- |
| `interpolator_example.py` | Plot jointspace spline vs. Cartesian pose interpolator output. | `models/iiwa_hanging.mjb` | `python examples/python/interpolator_example.py` |
| `sending_iiwa_ex.py` | Send jointspace + taskspace trajectories to an iiwa (sim or real). | `models/iiwa_hanging.mjb` | `python examples/python/sending_iiwa_ex.py` |
| `sending_kinova_ex.py` | Send jointspace commands to a Kinova Gen3 (simulation). | `models/kinova3.mjb` | `python examples/python/sending_kinova_ex.py` |
| `sending_linear_axis_ex.py` | Send trajectories to the linear axis in simulation. | `models/linear_axis.mjb` | `python examples/python/sending_linear_axis_ex.py` |
| `sending_param_ex.py` | Live-tune joint / Cartesian controller parameters via UDP. | `models/iiwa_hanging.mjb` | `python examples/python/sending_param_ex.py j` |
| `sending_robot9dof_ex.py` | Send trajectories to a combined 9-DOF (iiwa + linear axis) rig. | `models/iiwa_hanging.mjb`, `models/linear_axis.mjb` | `python examples/python/sending_robot9dof_ex.py` |
| `simulate_iiwa.py` | MuJoCo viewer loop that runs the iiwa controller from orcpy. | `models/iiwa_hanging.mjb` | `python examples/python/simulate_iiwa.py` |
| `simulate_kinova.py` | MuJoCo viewer loop for the Kinova controller. | `models/kinova3.mjb` | `python examples/python/simulate_kinova.py` |
| `simulate_linear_axis.py` | MuJoCo viewer loop for the linear axis. | `models/linear_axis.mjb` | `python examples/python/simulate_linear_axis.py` |
| `simulate_robot9dof.py` | MuJoCo viewer loop for the combined 9-DOF rig. | `models/presets/pascal_bernoulli.mjb`, plus iiwa + linear-axis MJBs | `python examples/python/simulate_robot9dof.py` |

Sub-folders `hybrid_force_motion/` and `orc_paper_experiments/` hold the
scripts used in the hybrid force/motion paper experiments. They operate on
the iiwa presets in `models/presets/` and require a running TwinCAT or
simulated server endpoint (see each script's header for details).

## Prerequisites

```bash
pip install -e .[examples]
```

Examples that talk to hardware expect the robot-side server running at the
IP/port configured at the top of each script; the simulation variants use
loop-back `127.0.0.1` and the ports exposed as
`oc.robots.<robot>.SERVER_PORT` / `CLIENT_PORT`.
