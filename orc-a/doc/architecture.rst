Architecture overview
=======================================================

This page describes how ORC is structured at runtime, what the control
loop looks like in practice, and what guarantees the library does — and
does **not** — make about timing and threading. It is intended as the
"big picture" companion to the per-module pages in :ref:`basic_components`.



Runtime model
-------------------------------------------------------

At the core sits an ``orc::robots::Robot`` (or one of its concrete
subclasses: ``Iiwa``, ``Kinova``, ``LinearAxis``, ``DummyRobot``). A
single control step looks like this:

1. The caller writes fresh measurements into the ``Robot`` (joint
   positions / velocities / torques, optionally end-effector pose).
2. The caller invokes ``robot.update(t)`` with the current time.
3. ``update`` advances the active trajectory from the
   ``TrajectoryQueue``, produces a desired setpoint ``(q_d, q_dot_d,
   …)``, and dispatches it to the registered controller(s).
4. The controller(s) compute a command (joint torque, joint velocity,
   …) from the tracking error.
5. The caller reads the resulting command out of the ``Robot`` and
   sends it to actuators (real hardware or simulator).

Steps 2–4 are pure: they don't touch I/O. I/O lives in ``orc::com``
(``TrajectoryServer``, ``TrajectorySerializer`` /
``TrajectoryDeserializer``) and is explicitly opt-in.


Module dependency diagram
-------------------------------------------------------

::

   ┌────────────┐      ┌────────────────────────┐     ┌──────────────┐
   │  orc::com  │◀────▶│    orc::robots::Robot  │────▶│ orc::control │
   │ (UDP / RT) │      │  (+ Iiwa / Kinova / …) │     │  controllers │
   └────────────┘      └──────┬─────────┬───────┘     └──────┬───────┘
                              │         │                    │
                              ▼         ▼                    ▼
                  ┌────────────┐   ┌──────────────┐   ┌────────────────┐
                  │ trajectory │   │ interpolator │   │ util (Logger,  │
                  │  (queue,   │◀──│  (spline,    │   │  Time, Angle,  │
                  │  js / ts)  │   │   cart pose) │   │  quatutil, …)  │
                  └─────┬──────┘   └──────┬───────┘   └────────────────┘
                        │                 │
                        └────────┬────────┘
                                 ▼
                         ┌────────────────┐
                         │ orc::RobotData │
                         │ (state block)  │
                         └────────────────┘

Only ``robots::Robot`` depends on everything; everything else can be
used standalone (and is, by the unit tests). MuJoCo is an implementation
detail of ``orc::robots`` — no other subsystem pulls it in.


Timing model
-------------------------------------------------------

ORC carries three explicit time scales, all expressed as
``orc::Time`` (nanosecond-accurate, see :doc:`basics/com`):

- **Ta** — the *trajectory sample time*. Time step at which a
  trajectory was originally sampled (produced by the user). Typical
  values: 1 ms – 10 ms.
- **Ts** — the *control sample time*. Time step at which
  ``robot.update(t)`` is called. Typical values: 100 µs – 1 ms.
- **t**  — the *wall-clock time* passed to ``update``. Monotonic,
  quantized to ``Ts`` via ``Time::quantize``.

Interpolators run at ``Ts`` and resample the ``Ta``-spaced trajectory
knots internally. ``Ts`` can be finer than ``Ta``; the reverse is a
user error.


Real-time expectations
-------------------------------------------------------

ORC is suitable for **soft real-time** control in simulation and in
cooperative scenarios (where an occasional deadline miss is survivable).
Concretely:

- The library is lock-free on the control path once trajectories are
  registered; no allocation happens in ``robot.update(t)`` for the
  default controllers.
- The logger is thread-safe but uses a mutex for reconfiguration; in
  production the logger level is set once at startup and then left
  alone.
- The UDP-based ``TrajectoryServer`` relies on the kernel socket
  buffer and does *not* provide bounded-latency guarantees.

**Hard real-time** (bounded worst-case jitter, no kernel page faults,
priority inheritance for any shared resource) requires the TwinCAT
integration maintained in the separate
`orc-iiwa-tc <https://github.com/niko-mit-d/orc-iiwa-tc>`_ repository,
which replaces the UDP transport and the standard allocator with
TwinCAT primitives.


Thread-safety notes
-------------------------------------------------------

- ``orc::log`` (the default logger) is safe to call from multiple
  threads; concurrent writes and concurrent ``start_logging()``
  reconfiguration are explicitly exercised by
  ``tests/logger_concurrency_test.cpp``.
- ``TrajectoryServer`` assumes **single-threaded** producer/consumer:
  exactly one thread calls ``start()`` / ``stop()`` / ``receive()``.
  It is not a general-purpose concurrent queue.
- ``Robot`` is not thread-safe. Register controllers and load the
  model up-front, then drive ``update(t)`` from a single control
  thread.
- ``TrajectoryQueue`` is not thread-safe. If you need to feed it from
  a network thread, synchronise externally (the TC integration does
  this via a TwinCAT-managed spin-lock).


.. _basic_components:

See also
-------------------------------------------------------

- :doc:`basics/trajectory` — trajectory types and the queue
- :doc:`basics/controller` — controller families and selection rules
- :doc:`basics/interpolator` — spline / pose interpolators
- :doc:`basics/com` — UDP transport and serializers
- :doc:`basics/robots` — concrete robot integrations
