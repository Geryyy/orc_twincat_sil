#pragma once

/**
 * @file FlatBuffers.h
 * @brief Main include file for FlatBuffers support in ORC
 *
 * Include this header to get access to all FlatBuffer serialization
 * and deserialization functionality.
 *
 * Usage:
 *   #include <orc/com/flatbuffers/FlatBuffers.h>
 *
 *   // Serialization (PC side)
 *   orc::com::fb::FlatBufferSerializer<7> serializer;
 *   auto data = serializer.serialize_joint_trajectory(time_pts, joint_pts);
 *
 *   // Deserialization (TwinCAT side)
 *   orc::com::fb::FlatBufferDeserializer<7> deserializer;
 *   auto reader = deserializer.get_joint_trajectory_reader(buffer, size);
 */

#ifndef ORC_FLATBUFFERS_H
#define ORC_FLATBUFFERS_H

#include "orc/util/import_flatbuffers.h"

#include "orc_messages_generated.h"

// Eigen <-> FlatBuffer scalar-vector glue
#include "orc/com/flatbuffers/FlatBufferEigen.h"

// Serializer (for building FlatBuffer messages)
#include "orc/com/flatbuffers/FlatBufferSerializer.h"

// Deserializer with zero-copy readers
#include "orc/com/flatbuffers/FlatBufferDeserializer.h"

// Robot state serialization
#include "orc/com/flatbuffers/FlatBufferRobotState.h"

#endif  // ORC_FLATBUFFERS_H
