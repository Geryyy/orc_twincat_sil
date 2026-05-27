#include <gtest/gtest.h>
#include <cstdint>
#include "orc/OrcTypes.h"
#include "orc/RobotStatus.h"
#include "orc/RobotTraits.h"
#include "orc/util/Time.h"
namespace {
// =========================================================================
// RobotTraits: type sizes for various DOF
// =========================================================================
TEST(RobotTraitsTest, JointVectorSizeDOF1) {
    using JV = orc::RobotTraits<1>::JointVector;
    EXPECT_EQ(sizeof(JV), 1 * sizeof(double));
    EXPECT_EQ(JV::RowsAtCompileTime, 1);
    EXPECT_EQ(JV::ColsAtCompileTime, 1);
}
TEST(RobotTraitsTest, JointVectorSizeDOF2) {
    using JV = orc::RobotTraits<2>::JointVector;
    EXPECT_EQ(sizeof(JV), 2 * sizeof(double));
    EXPECT_EQ(JV::RowsAtCompileTime, 2);
}
TEST(RobotTraitsTest, JointVectorSizeDOF7) {
    using JV = orc::RobotTraits<7>::JointVector;
    EXPECT_EQ(sizeof(JV), 7 * sizeof(double));
    EXPECT_EQ(JV::RowsAtCompileTime, 7);
}
TEST(RobotTraitsTest, JointMatrixSizeDOF7) {
    using JM = orc::RobotTraits<7>::JointMatrix;
    EXPECT_EQ(JM::RowsAtCompileTime, 7);
    EXPECT_EQ(JM::ColsAtCompileTime, 7);
}
TEST(RobotTraitsTest, JacobianMatrixSizeDOF7) {
    using JM = orc::RobotTraits<7>::JacobianMatrix;
    EXPECT_EQ(JM::RowsAtCompileTime, 6);
    EXPECT_EQ(JM::ColsAtCompileTime, 7);
}
TEST(RobotTraitsTest, JacobianInverseSizeDOF7) {
    using JIM = orc::RobotTraits<7>::JacobianInverseMatrix;
    EXPECT_EQ(JIM::RowsAtCompileTime, 7);
    EXPECT_EQ(JIM::ColsAtCompileTime, 6);
}
TEST(RobotTraitsTest, JointArraySizeDOF7) {
    using JA = orc::RobotTraits<7>::JointArray;
    EXPECT_EQ(JA::RowsAtCompileTime, 7);
}
// =========================================================================
// RobotStatus: enum size
// =========================================================================
TEST(RobotStatusTest, EnumSize) {
    // RobotStatus must be uint16_t for serialization
    EXPECT_EQ(sizeof(orc::logic::RobotStatus), sizeof(uint16_t));
}
TEST(RobotStatusTest, EnumValues) {
    EXPECT_EQ(static_cast<uint16_t>(orc::logic::RobotStatus::OFF), 0);
    EXPECT_EQ(static_cast<uint16_t>(orc::logic::RobotStatus::ENABLE), 1);
    EXPECT_EQ(static_cast<uint16_t>(orc::logic::RobotStatus::GRAVCOMP), 2);
}
// =========================================================================
// OrcTypes: vector sizes
// =========================================================================
TEST(ArcTypesTest, PoseVectorSize) {
    EXPECT_EQ(orc::PoseVector::RowsAtCompileTime, 7);
}
TEST(ArcTypesTest, CartesianVectorSize) {
    EXPECT_EQ(orc::CartesianVector::RowsAtCompileTime, 6);
}
TEST(ArcTypesTest, HybridVectorSize) {
    EXPECT_EQ(orc::HybridVector::RowsAtCompileTime, 8);
}
TEST(ArcTypesTest, HomogeneousTransformationSize) {
    EXPECT_EQ(orc::HomogeneousTransformation::RowsAtCompileTime, 4);
    EXPECT_EQ(orc::HomogeneousTransformation::ColsAtCompileTime, 4);
}
// =========================================================================
// Time packed struct size
// =========================================================================
TEST(PackedStructTest, TimeSizeIs16) {
    EXPECT_EQ(sizeof(orc::Time), 16u);
}
// NOTE: TrajectoryObject packed struct test removed — target uses FlatBuffers
// instead of the old packed TrajectoryObject struct.
}  // namespace
