#pragma once

/**
 * @file FlatBufferEigen.h
 * @brief Thin Eigen <-> FlatBuffer scalar-vector glue.
 *
 * Mirrors the pre-FlatBuffers writeEigenVector / readEigenVector helpers
 * (orc/com/SerializationFunctions.h, removed in commit 54ca447): a single
 * template body handles any DoF, with the wire payload as a length-prefixed
 * contiguous double array. The variadic schema (proto/orc_messages.fbs)
 * exposes joints as `[double]`, so write is one CreateVector call and read
 * is a zero-copy Eigen::Map over the buffer's contiguous storage.
 */

#include "orc/util/import_eigen.h"
#include "orc/util/import_flatbuffers.h"

namespace orc::com::fb {

template <typename Derived>
inline flatbuffers::Offset<flatbuffers::Vector<double>> fb_write_eigen(
    flatbuffers::FlatBufferBuilder& b, const Eigen::MatrixBase<Derived>& v) {
    static_assert(std::is_same<typename Derived::Scalar, double>::value,
                  "fb_write_eigen requires a double-valued Eigen vector");
    return b.CreateVector(v.derived().data(), static_cast<size_t>(v.size()));
}

template <int DOF>
inline bool fb_read_eigen(const flatbuffers::Vector<double>* src,
                          Eigen::Matrix<double, DOF, 1>& dst) {
    if (!src || src->size() != static_cast<flatbuffers::uoffset_t>(DOF))
        return false;
    dst = Eigen::Map<const Eigen::Matrix<double, DOF, 1>>(src->data());
    return true;
}

}  // namespace orc::com::fb
