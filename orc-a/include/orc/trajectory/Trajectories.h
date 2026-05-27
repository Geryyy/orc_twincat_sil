#include "orc/trajectory/TrajectoryBase.h"
#include "orc/trajectory/TrajectoryPointStorage.h"
#include "orc/trajectory/TrajectoryType.h"

/* normal trajectories */
// #include "orc/trajectory/JointspaceJerkTrajectory.h"
#include "orc/trajectory/JointspaceTrajectory.h"
// #include "orc/trajectory/TaskspaceJerkTrajectory.h"
#include "orc/trajectory/CartesianVelocityTrajectory.h"
#include "orc/trajectory/HybridForceMotionTrajectory.h"
#include "orc/trajectory/JointspaceVelocityTrajectory.h"
#include "orc/trajectory/TaskspaceTrajectory.h"

/* dense trajectories (no interpolation) */
#include "orc/trajectory/DenseJointspaceTrajectory.h"

/* dense trajectories (no interpolation) */
#include "orc/trajectory/DenseJointspaceTrajectory.h"

/* single event trajectories */
#include "orc/trajectory/singleevent/CartesianCtrParamTrajectory.h"
#include "orc/trajectory/singleevent/JointCtrParamTrajectory.h"
#include "orc/trajectory/singleevent/NullspaceTrajectory.h"

/* Queue */
#include "orc/trajectory/TrajectoryQueue.h"
