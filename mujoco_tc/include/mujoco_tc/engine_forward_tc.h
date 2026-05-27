#pragma once

#include "mujoco_tc/mjdata.h"
#include "mujoco_tc/mjmodel.h"


void mj_fwdPosition(const mjModel* m, mjData* d);
void mj_fwdVelocity(const mjModel* m, mjData* d);
void mj_forward(const mjModel* m, mjData* d);
void mj_addMDense(const mjModel* m, mjData* d, mjtNum* dst);
void mj_addMSparse(const mjModel* m, mjData* d, mjtNum* dst,
    int* rownnz, int* rowadr, int* colind, mjtNum* M,
    int* M_rownnz, int* M_rowadr, int* M_colind);