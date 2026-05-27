.. _com_submodule:

``orc::com`` submodule
==============================================

This submodule contains the communication layer between the host-side control loop and external senders/receivers (for example, trajectory streaming over UDP).

The module includes:

* trajectory and message identifiers (for example ``TrajectoryID``),
* communication settings and server utilities,
* FlatBuffers-based serialization/deserialization utilities under ``orc::com::fb``.

FlatBuffers in ORC
------------------

ORC uses FlatBuffers to encode trajectory and robot-state messages on the wire. The schema is defined in ``proto/orc_messages.fbs`` and compiled into ``include/orc/com/flatbuffers/orc_messages_generated.h``.

FlatBuffers are a good fit here because they:

- keep the wire format compact,
- support structured messages with explicit types,
- allow zero-copy access on the receiving side when the buffer is already trusted and verified,
- make it easy to keep the host-side sender and real-time receiver in sync through a shared schema.

Serialization and deserialization flow
--------------------------------------

A typical round-trip looks like this:

1. A sender builds a trajectory object or payload in memory.
2. ``orc::com::fb::FlatBufferSerializer`` converts the payload into a FlatBuffer message.
3. The message is transported over the communication channel, usually UDP.
4. On the receiving side, ``orc::com::fb::FlatBufferDeserializer`` verifies the buffer and checks the message type.
5. The receiver either reads the FlatBuffer data directly through the zero-copy reader helpers or converts it back into an ORC trajectory object.

The deserializer performs schema verification before reading the payload so malformed or incompatible data can be rejected early. This is especially important on the real-time side, where the receiver may want to inspect the incoming buffer without allocating or copying more data than necessary.

``orc::com::fb`` utilities
--------------------------

The FlatBuffers helper layer lives in ``include/orc/com/flatbuffers`` and provides the public entry points for message handling:

- ``FlatBufferSerializer`` builds FlatBuffer payloads for trajectories and robot-state messages.
- ``FlatBufferDeserializer`` verifies incoming buffers and either exposes zero-copy readers or reconstructs trajectory objects for the ``TrajectoryQueue``.
- ``FlatBuffers.h`` collects the FlatBuffers-specific types and conversion helpers used throughout the communication layer.

FlatBuffers header generation
-----------------------------

ORC provides a CMake option to regenerate FlatBuffers C++ headers from the schema:

* ``BUILD_FLATBUFFERS`` (default: ``OFF``)

Default behavior (recommended for normal builds):

* Keep the checked-in generated header.
* Build without regeneration.

Enable regeneration when the schema changes:

.. code-block:: bash

   cmake -S . -B build -DBUILD_FLATBUFFERS=ON
   cmake --build build -j

If the option is ``ON``, the build runs ``flatc`` and regenerates only the C++ FlatBuffers header used by ORC.

See also
--------

- :doc:`trajectory`
- :doc:`controller`
- :doc:`../architecture`
