#include "mujoco_tc/engine_forward_tc.h"

#include "mujoco_tc/mjdata.h"
#include "mujoco_tc/mjmodel.h"

#include "mujoco_tc/mujoco_tc.h"
#include "mujoco_tc/engine_core_smooth_tc.h"
#include "mujoco_tc/engine_util_misc_tc.h"
#include "mujoco_tc/engine_util_spatial_tc.h"
#include "mujoco_tc/engine_macro.h"
#include "mujoco_tc/engine_util_sparse_tc.h"
#include "mujoco_tc/engine_util_blas_tc.h"
#include "mujoco_tc/engine_core_constraint_tc.h"
#include "mujoco_tc/engine_passive_tc.h"
#include "mujoco_tc/engine_io_tc.h"
#include "mujoco_tc/engine_collision_driver_tc.h"
#include "mujoco_tc/engine_util_solve_tc.h"


// forward dynamics with skip; skipstage is mjtStage
void mj_forwardSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor) {
    TM_START;

    mj_fwdPosition(m, d);
    mj_fwdVelocity(m, d);

    // qfrc_smooth = qfrc_applied + qfrc_passive - qfrc_bias
    int nv = m->nv;
    mju_sub(d->qfrc_smooth, d->qfrc_applied, d->qfrc_bias, nv);
    mju_addTo(d->qfrc_smooth, d->qfrc_passive, nv);

    // solve M * qacc = qfrc_smooth via dense Cholesky
    mj_markStack(d);
    mjtNum* Mdense = mjSTACKALLOC(d, nv * nv, mjtNum);
    mj_fullM(m, Mdense, d->qM);
    mju_cholFactor(Mdense, nv, 0);
    mju_cholSolve(d->qacc, Mdense, d->qfrc_smooth, nv);
    mju_copy(d->qacc_smooth, d->qacc, nv);
    mj_freeStack(d);

    TM_END(mjTIMER_FORWARD);
}


// forward dynamics
void mj_forward(const mjModel* m, mjData* d) {
    mj_forwardSkip(m, d, mjSTAGE_NONE, 0);
}


// TC: Rewritten and minimized
// position-dependent computations
void mj_fwdPosition(const mjModel* m, mjData* d) {
    TM_START1;

    TM_START;
    mj_kinematics(m, d);
    mj_comPos(m, d);
    TM_END(mjTIMER_POS_KINEMATICS);

    // no threadpool: inertia and collision on main thread
    //if (!d->threadpool) {
        mj_crb(m, d);        // timed internally (POS_INERTIA)
        // mj_tendonArmature(m, d);  // timed internally (POS_INERTIA)
        mj_factorM(m, d);    // timed internally (POS_INERTIA)
        mj_collision(m, d);  // timed internally (POS_COLLISION)
    //}

    //TM_RESTART;
    //mj_makeConstraint(m, d);
    //if (mjENABLED(mjENBL_ISLAND)) {
    //    mj_island(m, d);
    //}
    //TM_END(mjTIMER_POS_MAKE);

    //TM_RESTART;
    //mj_transmission(m, d);
    //TM_ADD(mjTIMER_POS_KINEMATICS);

    //TM_RESTART;
    //mj_projectConstraint(m, d);
    //TM_END(mjTIMER_POS_PROJECT);

    //TM_END1(mjTIMER_POSITION);
}

// velocity-dependent computations
void mj_fwdVelocity(const mjModel* m, mjData* d) {
    TM_START;

    // flexedge velocity: dense or sparse
    if (mj_isSparse(m)) {
        mju_mulMatVecSparse(d->flexedge_velocity, d->flexedge_J, d->qvel, m->nflexedge,
            d->flexedge_J_rownnz, d->flexedge_J_rowadr, d->flexedge_J_colind, NULL);
    }
    else {
        mju_mulMatVec(d->flexedge_velocity, d->flexedge_J, d->qvel, m->nflexedge, m->nv);
    }

    // tendon velocity: dense or sparse
    if (mj_isSparse(m)) {
        mju_mulMatVecSparse(d->ten_velocity, d->ten_J, d->qvel, m->ntendon,
            d->ten_J_rownnz, d->ten_J_rowadr, d->ten_J_colind, NULL);
    }
    else {
        mju_mulMatVec(d->ten_velocity, d->ten_J, d->qvel, m->ntendon, m->nv);
    }

    // actuator velocity: always sparse
    if (!mjDISABLED(mjDSBL_ACTUATION)) {
        mju_mulMatVecSparse(d->actuator_velocity, d->actuator_moment, d->qvel, m->nu,
            d->moment_rownnz, d->moment_rowadr, d->moment_colind, NULL);
    }

    // com-based velocities, passive forces, constraint references
    mj_comVel(m, d);
    mj_passive(m, d);
    mj_referenceConstraint(m, d);

    // compute qfrc_bias with abbreviated RNE (without acceleration)
    mj_rne(m, d, 0, d->qfrc_bias);

    // // add bias force due to tendon armature
    // mj_tendonBias(m, d, d->qfrc_bias);

    TM_END(mjTIMER_VELOCITY);
}


//-------------------------- inertia functions -----------------------------------------------------

// convert sparse inertia matrix M into full matrix
void mj_fullM(const mjModel* m, mjtNum* dst, const mjtNum* M) {
    int adr = 0, nv = m->nv;
    mju_zero(dst, nv * nv);

    for (int i = 0; i < nv; i++) {
        int j = i;
        while (j >= 0) {
            dst[i * nv + j] = M[adr];
            dst[j * nv + i] = M[adr];
            j = m->dof_parentid[j];
            adr++;
        }
    }
}



// multiply vector by inertia matrix
void mj_mulM(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec) {
    int nv = m->nv;
    const mjtNum* M = d->qM;
    const int* Madr = m->dof_Madr;
    const int* parentid = m->dof_parentid;
    const int* simplenum = m->dof_simplenum;

    mju_zero(res, nv);

    for (int i = 0; i < nv; i++) {
#ifdef mjUSEAVX
        // simple: diagonal multiplication, AVX
        if (simplenum[i] >= 4) {
            // init
            __m256d result, val1, val2;

            // parallel computation
            val1 = _mm256_loadu_pd(vec + i);
            val2 = _mm256_set_pd(M[Madr[i + 3]],
                M[Madr[i + 2]],
                M[Madr[i + 1]],
                M[Madr[i + 0]]);
            result = _mm256_mul_pd(val1, val2);

            // store result
            _mm256_storeu_pd(res + i, result);

            // skip rest of block
            i += 3;
            continue;
        }
#endif
        // address in M
        int adr = Madr[i];

        // compute diagonal
        res[i] = M[adr] * vec[i];

        // simple dof: continue
        if (simplenum[i]) {
            continue;
        }

        // compute off-diagonals
        int j = parentid[i];
        while (j >= 0) {
            adr++;
            res[i] += M[adr] * vec[j];
            res[j] += M[adr] * vec[i];

            // advance to parent
            j = parentid[j];
        }
    }
}



// multiply vector by inertia matrix for one dof island
void mj_mulM_island(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec,
    int island, int flg_vecunc) {
    // if no island, call regular function
    if (island < 0) {
        mj_mulM(m, d, res, vec);
        return;
    }

    // local constants: general
    const mjtNum* M = d->qM;
    const int* Madr = m->dof_Madr;
    const int* parentid = m->dof_parentid;
    const int* simplenum = m->dof_simplenum;

    // local constants: island specific
    int ndof = d->island_dofnum[island];
    const int* dofind = d->island_dofind + d->island_dofadr[island];
    const int* islandind = d->dof_islandind;

    mju_zero(res, ndof);

    for (int k = 0; k < ndof; k++) {
        // address in full dof vector
        int i = dofind[k];

        // address in M
        int adr = Madr[i];

        // diagonal
        if (flg_vecunc) {
            res[k] = M[adr] * vec[i];
        }
        else {
            res[k] = M[adr] * vec[k];
        }

        // simple dof: continue
        if (simplenum[i]) {
            continue;
        }

        // off-diagonal
        int j = parentid[i];
        while (j >= 0) {
            adr++;
            int l = islandind[j];
            if (flg_vecunc) {
                res[k] += M[adr] * vec[j];
                res[l] += M[adr] * vec[i];
            }
            else {
                res[k] += M[adr] * vec[l];
                res[l] += M[adr] * vec[k];
            }

            // advance to parent
            j = parentid[j];
        }
    }
}



// multiply vector by M^(1/2)
void mj_mulM2(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec) {
    int  nv = m->nv;
    const mjtNum* qLD = d->qLD;
  
    mju_zero(res, nv);
  
    // res = L * vec
    for (int i=0; i < nv; i++) {
      // diagonal
      res[i] = vec[i];
  
      // non-simple: add off-diagonals
      if (!m->dof_simplenum[i]) {
        int adr = d->M_rowadr[i];
        res[i] += mju_dotSparse(qLD+adr, vec, d->M_rownnz[i] - 1, d->M_colind+adr, /*flg_unc1=*/0);
      }
    }
  
    // res *= sqrt(D)
    for (int i=0; i < nv; i++) {
      int diag = d->M_rowadr[i] + d->M_rownnz[i] - 1;
      res[i] *= mju_sqrt(qLD[diag]);
    }
  }
  

  
// add inertia matrix to destination matrix
//  destination can be sparse uncompressed, or dense when all int* are NULL
void mj_addM(const mjModel* m, mjData* d, mjtNum* dst,
    int* rownnz, int* rowadr, int* colind) {
// sparse
if (rownnz && rowadr && colind) {
int nC = m->nC;
mj_markStack(d);

// gather C <- qM (legacy to CSR)
mjtNum* C = mjSTACKALLOC(d, nC, mjtNum);
mju_gather(C, d->qM, d->mapM2C, nC);

// add to dst
mj_addMSparse(m, d, dst, rownnz, rowadr, colind, C, d->C_rownnz, d->C_rowadr, d->C_colind);
mj_freeStack(d);
}

// dense
else {
mj_addMDense(m, d, dst);
}
}



// add inertia matrix to sparse destination matrix
void mj_addMSparse(const mjModel* m, mjData* d, mjtNum* dst,
    int* rownnz, int* rowadr, int* colind, mjtNum* M,
    int* M_rownnz, int* M_rowadr, int* M_colind) {
int nv = m->nv;

mj_markStack(d);
int* buf_ind = mjSTACKALLOC(d, nv, int);
mjtNum* sparse_buf = mjSTACKALLOC(d, nv, mjtNum);

// add to destination
for (int i=0; i < nv; i++) {
rownnz[i] = mju_combineSparse(dst + rowadr[i], M + M_rowadr[i], 1, 1,
                   rownnz[i], M_rownnz[i], colind + rowadr[i],
                   M_colind + M_rowadr[i], sparse_buf, buf_ind);
}
mj_freeStack(d);
}



// add inertia matrix to dense destination matrix
void mj_addMDense(const mjModel* m, mjData* d, mjtNum* dst) {
    int nv = m->nv;

    for (int i = 0; i < nv; i++) {
        int adr = m->dof_Madr[i];
        int j = i;
        while (j >= 0) {
            // add
            dst[i * nv + j] += d->qM[adr];
            if (j < i) {
                dst[j * nv + i] += d->qM[adr];
            }

            // only diagonal if simplenum
            if (m->dof_simplenum[i]) {
                break;
            }

            // advance
            j = m->dof_parentid[j];
            adr++;
        }
    }
}


