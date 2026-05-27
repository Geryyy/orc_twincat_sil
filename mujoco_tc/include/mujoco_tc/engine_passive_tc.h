#pragma once

#include "mujoco_tc/mjdata.h"
#include "mujoco_tc/mjmodel.h"

static void inline GradSquaredLengths(mjtNum gradient[6][2][3],
    const mjtNum* xpos,
    const int vert[4],
    const int edge[6][2],
    int nedge);
static void mj_springdamper(const mjModel* m, mjData* d);
void mj_passive(const mjModel* m, mjData* d);