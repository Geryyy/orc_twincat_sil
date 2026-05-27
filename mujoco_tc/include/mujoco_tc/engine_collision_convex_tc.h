#pragma once 

#include "mujoco_tc/mjmodel.h"
#include "mujoco_tc/mjdata.h"
#include "mujoco_tc/mjtnum.h"

// TC: Emptied function. To be implemented!
// geom-elem or elem-elem or vert-elem collision function using ccd
inline int mjc_ConvexElem(const mjModel* m, const mjData* d, mjContact* con,
                       int g1, int f1, int e1, int v1, int f2, int e2, mjtNum margin){
    return 0;
}

// heightfield-elem collision function using ccd
inline int mjc_HFieldElem    (const mjModel* m, const mjData* d, mjContact* con,
                       int g, int f, int e, mjtNum margin){
    return 0;                    
}