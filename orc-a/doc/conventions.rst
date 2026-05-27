Unit and sign conventions
=======================================================

ORC's public API takes plain ``double`` and ``Eigen::Vector*`` types;
there are no typed wrappers (yet) to guard against unit mix-ups. This
page records the conventions the library assumes. Callers are
responsible for respecting them — trajectory and controller pages cite
this file rather than restating units inline.

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Quantity
     - Unit
     - Notes
   * - Angle (joint position, orientation)
     - rad
     - Joint positions are wrapped to ``(-π, π]`` internally by
       :cpp:func:`orc::util::wrap_to_pi` inside joint controllers.
   * - Angular velocity / acceleration
     - rad/s, rad/s²
     -
   * - Length (Cartesian position)
     - m
     -
   * - Linear velocity / acceleration
     - m/s, m/s²
     -
   * - Time
     - s
     - Internally represented by :cpp:class:`orc::Time` (nanosecond
       resolution). ``Time::toSec()`` returns a ``double`` in seconds.
   * - Force
     - N
     - Applied at an end-effector site.
   * - Torque (joint / wrench)
     - N·m
     - Joint torque is the default controller output.
   * - Quaternion
     - unit, ``(w, x, y, z)``
     - Hamilton convention (scalar-first). Stored in
       :cpp:class:`Eigen::Quaterniond` and as
       :cpp:type:`orc::PoseVector` entries
       ``(x, y, z, q_w, q_x, q_y, q_z)``.

Sign conventions
-------------------------------------------------------

- Joint errors are computed as ``q_act - q_d`` and then wrapped to
  ``(-π, π]`` before being multiplied by the controller gain.
- Torque sign follows the model's joint axis (right-hand rule around
  the joint axis vector in the MJCF model).
- End-effector pose is expressed in the world frame unless a specific
  controller documents otherwise.
