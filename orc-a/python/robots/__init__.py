"""High-level Python robot wrappers on top of the compiled ``orcpy.core`` bindings.

The package groups classes into two tiers:

**Base classes — subclass these only if you are adding a new robot type.**
    * :class:`Robot` — abstract UDP + serializer plumbing shared by every robot
      wrapper. Not meant to be instantiated directly.
    * :class:`Robot9DOF` — composite wrapper that glues an :class:`Iiwa` on top
      of a :class:`LinearAxis`; treat this as a ready-to-use rig, not as a
      general-purpose base.

**End-user classes — instantiate these to talk to a robot.**
    * :class:`Iiwa` — KUKA LBR iiwa (7 DoF).
    * :class:`Kinova` — Kinova Gen3 (7 DoF).
    * :class:`LinearAxis` — 2-DoF linear axis stage.

Utility helpers live in :mod:`orcpy.robots.util_functions`
(``default_model_path``, ``inverse_kinematics_residual``, ``wrap_to_pi`` …).

Typical usage::

    import orcpy.robots as orco
    iiwa = orco.Iiwa.from_default_config(simulation=True)
"""

# Import primitive robot class derivations.
# Import util functions
from . import util_functions
from .Iiwa import Iiwa
from .Kinova import Kinova
from .LinearAxis import LinearAxis
from .Robot import Robot

# Import wrapper robot classes, i.e., ones that unify multiple primitive types.
from .Robot9DOF import Robot9DOF
from .util_functions import default_model_path, inverse_kinematics_residual

__all__ = [
    "Iiwa",
    "Kinova",
    "LinearAxis",
    "Robot",
    "Robot9DOF",
    "default_model_path",
    "inverse_kinematics_residual",
    "util_functions",
]
