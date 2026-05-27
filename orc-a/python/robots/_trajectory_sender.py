"""Internal helper that centralises the serialize-and-UDP-split loop used by
:class:`Robot` for every ``send_*_trajectory`` method.

The class is intentionally not re-exported from :mod:`orcpy.robots`: it is an
implementation detail of :class:`orcpy.robots.Robot`, but lives in its own
module so the god-object gets smaller and the sending path becomes testable
in isolation.
"""

from __future__ import annotations

import socket
import time
from collections.abc import Iterable
from typing import Any


class TrajectorySender:
    """Serialize a trajectory via ``serializer`` and stream the splits over UDP.

    Split serializers return a ``list[bytes]`` (one UDP datagram per
    element) -- the ``_split`` variants split long trajectories at the
    sender to respect the 1472 B UDP limit and bound the receiver's
    spline-init cost. Single-buffer serializers (one-shot messages like
    nullspace / ctrparam / stop) return a single ``bytes``.
    """

    # Small inter-packet sleep so the robot controller can drain its buffer.
    _PACKET_GAP_S = 1e-3

    def __init__(
        self,
        serializer: Any,
        module: Any,
        udp_socket: socket.socket,
        robot_address: tuple[str, int],
    ) -> None:
        self.serializer = serializer
        self.module = module
        self._udp_socket = udp_socket
        self._robot_address = robot_address

    @property
    def robot_address(self) -> tuple[str, int]:
        return self._robot_address

    @robot_address.setter
    def robot_address(self, addr: tuple[str, int]) -> None:
        self._robot_address = addr

    def _send_many(self, buffers: Iterable[Any]) -> None:
        for buf in buffers:
            self._udp_socket.sendto(bytes(buf), self._robot_address)
            time.sleep(self._PACKET_GAP_S)

    def _send_one(self, buf: Any) -> None:
        self._udp_socket.sendto(bytes(buf), self._robot_address)

    # --- Typed wrappers for each serializer flavour --------------------

    def send_jointspace_trajectory(self, time_pts, joint_pts) -> None:
        self._send_many(self.serializer.serialize_joint_trajectory_split(time_pts, joint_pts))

    def send_taskspace_trajectory(self, time_pts, pose_pts) -> None:
        self._send_many(self.serializer.serialize_cartesian_trajectory_split(time_pts, pose_pts))

    def send_cartesian_velocity_trajectory(self, time_pts, velocity_pts) -> None:
        self._send_many(
            self.serializer.serialize_cartesian_velocity_trajectory_split(time_pts, velocity_pts)
        )

    def send_jointspace_velocity_trajectory(self, time_pts, velocity_pts) -> None:
        self._send_many(
            self.serializer.serialize_jointspace_velocity_trajectory_split(time_pts, velocity_pts)
        )

    def send_joint_ctr_parameter_trajectory(self, time_pts, K0, K1, KI) -> None:
        param = self.module.JointCTParameter(K0, K1, KI)
        self._send_one(self.serializer.serialize_jointctrparam_trajectory(time_pts, param))

    def send_cartesian_ctr_parameter_trajectory(self, time_pts, K0, K1, K0N, K1N) -> None:
        param = self.module.CartesianCTParameter(K0, K1, K0N, K1N)
        self._send_one(self.serializer.serialize_cartesianctrparam_trajectory(time_pts, param))

    def send_hybrid_force_motion_trajectory(self, pose_pts, force_pts, time_pts) -> None:
        self._send_many(
            self.serializer.serialize_hybrid_force_motion_trajectory_split(
                time_pts, pose_pts, force_pts
            )
        )

    def send_nullspace_trajectory(self, time_pts, nullspace_joint_state) -> None:
        self._send_one(
            self.serializer.serialize_nullspace_trajectory(time_pts, nullspace_joint_state)
        )
