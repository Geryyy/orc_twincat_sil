#pragma once

#include "mujoco_tc/mujoco_tc.h"

void mj_kinematics(const mjModel* m, mjData* d);
void mj_comPos(const mjModel* m, mjData* d);
void mj_crb(const mjModel* m, mjData* d);
void mj_factorM(const mjModel* m, mjData* d);
void mj_comVel(const mjModel* m, mjData* d);
// sparse L'*D*L factorizaton of inertia-like matrix M, assumed spd  (legacy implementation)
void mj_factorI_legacy(const mjModel* m, mjData* d, const mjtNum* M,
    mjtNum* qLD, mjtNum* qLDiagInv);
// sparse L'*D*L factorizaton of inertia-like matrix
void mj_factorI(mjtNum* mat, mjtNum* diaginv, int nv,
    const int* rownnz, const int* rowadr, const int* diagnum, const int* colind);


//-------------------------- RNE -------------------------------------------------------------------

// RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term
void mj_rne(const mjModel* m, mjData* d, int flg_acc, mjtNum* result);
