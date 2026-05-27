#pragma once

#include "mujoco_tc/mjdata.h"
#include "mujoco_tc/mjmodel.h"
#include "mujoco_tc/mjtnum.h"

void mj_local2Global(mjData* d, mjtNum xpos[3], mjtNum xmat[9],
    const mjtNum pos[3], const mjtNum quat[4],
    int body, mjtByte sameframe);
void mj_warning(mjData* d, int warning, int info);

//-------------------------- sparse chains ---------------------------------------------------------

// merge dof chains for two bodies
int mj_mergeChain(const mjModel* m, int* chain, int b1, int b2);

// merge dof chains for two simple bodies
int mj_mergeChainSimple(const mjModel* m, int* chain, int b1, int b2);

// get body chain
int mj_bodyChain(const mjModel* m, int body, int* chain);


void mj_applyFT(const mjModel* m, mjData* d,
    const mjtNum force[3], const mjtNum torque[3],
    const mjtNum point[3], int body, mjtNum* qfrc_target);

//-------------------------- Jacobians -------------------------------------------------------------

// compute 3/6-by-nv Jacobian of global point attached to given body
void mj_jac(const mjModel* m, const mjData* d,
                  mjtNum* jacp, mjtNum* jacr, const mjtNum point[3], int body);

// compute body frame Jacobian
void mj_jacBody(const mjModel* m, const mjData* d,
                      mjtNum* jacp, mjtNum* jacr, int body);

// compute body center-of-mass Jacobian
void mj_jacBodyCom(const mjModel* m, const mjData* d,
                         mjtNum* jacp, mjtNum* jacr, int body);

// compute subtree center-of-mass Jacobian
void mj_jacSubtreeCom(const mjModel* m, mjData* d, mjtNum* jacp, int body);

// compute geom Jacobian
void mj_jacGeom(const mjModel* m, const mjData* d,
                      mjtNum* jacp, mjtNum* jacr, int geom);

// compute site Jacobian
void mj_jacSite(const mjModel* m, const mjData* d,
                      mjtNum* jacp, mjtNum* jacr, int site);

// compute translation Jacobian of point, and rotation Jacobian of axis
void mj_jacPointAxis(const mjModel* m, mjData* d,
                           mjtNum* jacPoint, mjtNum* jacAxis,
                           const mjtNum point[3], const mjtNum axis[3], int body);

// compute 3/6-by-nv sparse Jacobian of global point attached to given body
void mj_jacSparse(const mjModel* m, const mjData* d,
                  mjtNum* jacp, mjtNum* jacr, const mjtNum* point, int body,
                  int NV, const int* chain);

// sparse Jacobian difference for simple body contacts
void mj_jacSparseSimple(const mjModel* m, const mjData* d,
                        mjtNum* jacdifp, mjtNum* jacdifr, const mjtNum* point,
                        int body, int flg_second, int NV, int start);

// dense or sparse Jacobian difference for two body points: pos2 - pos1, global
int mj_jacDifPair(const mjModel* m, const mjData* d, int* chain,
                        int b1, int b2, const mjtNum pos1[3], const mjtNum pos2[3],
                        mjtNum* jac1p, mjtNum* jac2p, mjtNum* jacdifp,
                        mjtNum* jac1r, mjtNum* jac2r, mjtNum* jacdifr);

// dense or sparse weighted sum of multiple body Jacobians at same point
int mj_jacSum(const mjModel* m, mjData* d, int* chain,
              int n, const int* body, const mjtNum* weight,
              const mjtNum point[3], mjtNum* jac, int flg_rot);

// compute 3/6-by-nv Jacobian time derivative of global point attached to given body
void mj_jacDot(const mjModel* m, const mjData* d,
                     mjtNum* jacp, mjtNum* jacr, const mjtNum point[3], int body);

// compute subtree angular momentum matrix
void mj_angmomMat(const mjModel* m, mjData* d, mjtNum* mat, int body);


//-------------------------- inertia functions -----------------------------------------------------

// convert sparse inertia matrix M into full matrix
void mj_fullM(const mjModel* m, mjtNum* dst, const mjtNum* M);

// multiply vector by inertia matrix
void mj_mulM(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);

// multiply vector by inertia matrix for one dof island
void mj_mulM_island(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec,
    int island, int flg_vecunc);

// multiply vector by (inertia matrix)^(1/2)
void mj_mulM2(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);

// add inertia matrix to destination matrix
//  destination can be sparse uncompressed, or dense when all int* are NULL
void mj_addM(const mjModel* m, mjData* d, mjtNum* dst,
    int* rownnz, int* rowadr, int* colind);

// add inertia matrix to sparse destination matrix
void mj_addMSparse(const mjModel* m, mjData* d, mjtNum* dst,
    int* rownnz, int* rowadr, int* colind, mjtNum* M,
    int* M_rownnz, int* M_rowadr, int* M_colind);

// add inertia matrix to dense destination matrix
void mj_addMDense(const mjModel* m, mjData* d, mjtNum* dst);

//-------------------------- miscellaneous ---------------------------------------------------------

// integrate qpos with given qvel
void mj_integratePos(const mjModel* m, mjtNum* qpos, const mjtNum* qvel, mjtNum dt);