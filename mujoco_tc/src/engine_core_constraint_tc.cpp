#include "mujoco_tc/mjmodel.h"
#include "mujoco_tc/mjdata.h"
#include "mujoco_tc/engine_support_tc.h"
#include "mujoco_tc/engine_util_sparse_tc.h"
#include "mujoco_tc/engine_util_blas_tc.h"
#include "mujoco_tc/engine_util_misc_tc.h"
#include "mujoco_tc/mujoco_tc.h"


// determine type of friction cone
int mj_isPyramidal(const mjModel* m) {
    if (m->opt.cone == mjCONE_PYRAMIDAL) {
        return 1;
    }
    else {
        return 0;
    }
}



// determine type of constraint Jacobian
int mj_isSparse(const mjModel* m) {
    if (m->opt.jacobian == mjJAC_SPARSE ||
        (m->opt.jacobian == mjJAC_AUTO && m->nv >= 60)) {
        return 1;
    }
    else {
        return 0;
    }
}



// determine type of solver
int mj_isDual(const mjModel* m) {
    if (m->opt.solver == mjSOL_PGS || m->opt.noslip_iterations > 0) {
        return 1;
    }
    else {
        return 0;
    }
}



// assign/clamp contact friction parameters
void mj_assignFriction(const mjModel* m, mjtNum* target, const mjtNum* source) {
    if (mjENABLED(mjENBL_OVERRIDE)) {
        for (int i = 0; i < 5; i++) {
            target[i] = mju_max(mjMINMU, m->opt.o_friction[i]);
        }
    }
    else {
        for (int i = 0; i < 5; i++) {
            target[i] = mju_max(mjMINMU, source[i]);
        }
    }
}



// assign/override contact reference parameters
void mj_assignRef(const mjModel* m, mjtNum* target, const mjtNum* source) {
    if (mjENABLED(mjENBL_OVERRIDE)) {
        mju_copy(target, m->opt.o_solref, mjNREF);
    }
    else {
        mju_copy(target, source, mjNREF);
    }
}



// assign/override contact impedance parameters
void mj_assignImp(const mjModel* m, mjtNum* target, const mjtNum* source) {
    if (mjENABLED(mjENBL_OVERRIDE)) {
        mju_copy(target, m->opt.o_solimp, mjNIMP);
    }
    else {
        mju_copy(target, source, mjNIMP);
    }
}



// assign/override contact margin
mjtNum mj_assignMargin(const mjModel* m, mjtNum source) {
    if (mjENABLED(mjENBL_OVERRIDE)) {
        return m->opt.o_margin;
    }
    else {
        return source;
    }
}



// compute element bodies and weights for given contact point, return #bodies
// if v is one of the element vertices, reduce element to fragment
static int mj_elemBodyWeight(const mjModel* m, const mjData* d, int f, int e, int v,
    const mjtNum point[3], int* body, mjtNum* weight) {
// get flex info
int dim = m->flex_dim[f];
const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim+1);
const mjtNum* vert = d->flexvert_xpos + 3*m->flex_vertadr[f];

// compute inverse distances from contact point to element vertices
// save body ids, find vertex v in element
int vid = -1;
for (int i=0; i <= dim; i++) {
mjtNum dist = mju_dist3(point, vert+3*edata[i]);
weight[i] = 1.0/(mju_max(mjMINVAL, dist));
body[i] = m->flex_vertadr[f] + edata[i];

// check if element vertex matches v
if (edata[i] == v) {
vid = i;
}
}

// v found in e: skip and shift remaining
if (vid >= 0) {
while (vid < dim) {
weight[vid] = weight[vid+1];
body[vid] = body[vid+1];
vid++;
}
dim--;
}

// normalize weights
mju_normalize(weight, dim+1);
return dim+1;
}



// compute body weights for a given contact vertex, return #bodies
static int mj_vertBodyWeight(const mjModel* m, const mjData* d, int f, int v,
    const mjtNum point[3], int* body, mjtNum* weight, mjtNum bw) {
mjtNum* coord = m->flex_vert0 + 3*v;
int nstart = m->flex_nodeadr[f];
int nend = m->flex_nodeadr[f] + m->flex_nodenum[f];
int nb = 0;

for (int i = nstart; i < nend; i++) {
mjtNum w = ((i-nstart)&1 ? coord[2] : 1-coord[2]) *
((i-nstart)&2 ? coord[1] : 1-coord[1]) *
((i-nstart)&4 ? coord[0] : 1-coord[0]);
if (w < 1e-5) {
continue;
}
if (weight) weight[nb] = w * bw;
body[nb++] = m->flex_nodebodyid[i];
}

return nb;
}


// TC: Change __Alignof(type) to 1 and added static_cast
// add contact to d->contact list; return 0 if success; 1 if buffer full
int mj_addContact(const mjModel* m, mjData* d, const mjContact* con) {
   // if nconmax is specified and ncon >= nconmax, warn and return error
   if (m->nconmax != -1 && d->ncon >= m->nconmax) {
       mj_warning(d, mjWARN_CONTACTFULL, d->ncon);
       return 1;
   }

   // move arena pointer back to the end of the existing contact array and invalidate efc_ arrays
   d->parena = d->ncon * sizeof(mjContact);
#ifdef ADDRESS_SANITIZER
   ASAN_POISON_MEMORY_REGION(
       (char*)d->arena + d->parena, d->narena - d->pstack - d->parena);
#endif
   mj_clearEfc(d);

   // copy contact
   mjContact* dst = static_cast<mjContact *>(mj_arenaAllocByte(d, sizeof(mjContact), 1));
   if (!dst) {
       mj_warning(d, mjWARN_CONTACTFULL, d->ncon);
       return 1;
   }
   *dst = *con;

   // increase counter, return success
   d->ncon++;
   return 0;
}



// add #size rows to constraint Jacobian; set pos, margin, frictionloss, type, id
static void mj_addConstraint(const mjModel* m, mjData* d,
    const mjtNum* jac, const mjtNum* pos,
    const mjtNum* margin, mjtNum frictionloss,
    int size, int type, int id, int NV, const int* chain) {
    int empty, nv = m->nv, nefc = d->nefc;
    int* nnz = d->efc_J_rownnz, * adr = d->efc_J_rowadr, * ind = d->efc_J_colind;
    mjtNum* J = d->efc_J;

    // init empty guard for constraints other than contact
    if (type == mjCNSTR_CONTACT_FRICTIONLESS ||
        type == mjCNSTR_CONTACT_PYRAMIDAL ||
        type == mjCNSTR_CONTACT_ELLIPTIC) {
        empty = 0;
    }
    else {
        empty = 1;
    }

    // dense: copy entire Jacobian
    if (!mj_isSparse(m)) {
        // make sure jac is not empty
        if (empty) {
            for (int i = 0; i < size * nv; i++) {
                if (jac[i]) {
                    empty = 0;
                    break;
                }
            }
        }

        // copy if not empty
        if (!empty) {
            mju_copy(J + nefc * nv, jac, size * nv);
        }
    }

    // sparse: copy chain
    else {
        // clamp NV (in case -1 was used in constraint construction)
        NV = mjMAX(0, NV);

        if (NV) {
            empty = 0;
        }
        else if (empty) {
            // all rows are empty, return early
            return;
        }

        // chain required in sparse mode
        if (NV && !chain) {
            mjERROR("called with dense arguments");
        }

        // process size elements
        for (int i = 0; i < size; i++) {
            // set row address
            adr[nefc + i] = (nefc + i ? adr[nefc + i - 1] + nnz[nefc + i - 1] : 0);

            // set row descriptor
            nnz[nefc + i] = NV;

            // copy if not empty
            if (NV) {
                mju_copyInt(ind + adr[nefc + i], chain, NV);
                mju_copy(J + adr[nefc + i], jac + i * NV, NV);
            }
        }
    }

    // all rows empty: skip constraint
    if (empty) {
        return;
    }

    // set constraint pos, margin, frictionloss, type, id
    for (int i = 0; i < size; i++) {
        d->efc_pos[nefc + i] = (pos ? pos[i] : 0);
        d->efc_margin[nefc + i] = (margin ? margin[i] : 0);
        d->efc_frictionloss[nefc + i] = frictionloss;
        d->efc_type[nefc + i] = type;
        d->efc_id[nefc + i] = id;
    }

    // increase counters
    d->nefc += size;
    if (type == mjCNSTR_EQUALITY) {
        d->ne += size;
    }
    else if (type == mjCNSTR_FRICTION_DOF || type == mjCNSTR_FRICTION_TENDON) {
        d->nf += size;
    }
    else if (type == mjCNSTR_LIMIT_JOINT || type == mjCNSTR_LIMIT_TENDON) {
        d->nl += size;
    }
}



// multiply Jacobian by vector
void mj_mulJacVec(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec) {
    // exit if no constraints
    if (!d->nefc) {
        return;
    }

    // sparse Jacobian
    if (mj_isSparse(m))
        mju_mulMatVecSparse(res, d->efc_J, vec, d->nefc,
            d->efc_J_rownnz, d->efc_J_rowadr,
            d->efc_J_colind, d->efc_J_rowsuper);

    // dense Jacobian
    else {
        mju_mulMatVec(res, d->efc_J, vec, d->nefc, m->nv);
    }
}



// multiply Jacobian by vector, for one island
//  flg_resunc and flg_vecunc denote whether res/vec are uncompressed
void mj_mulJacVec_island(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec,
    int island, int flg_resunc, int flg_vecunc) {
    // no island, call regular function
    if (island < 0) {
        mj_mulJacVec(m, d, res, vec);
        return;
    }

    // sizes
    int vecnnz = d->island_dofnum[island];
    int resnnz = d->island_efcnum[island];

    // indices
    int* vecind = d->island_dofind + d->island_dofadr[island];
    int* resind = d->island_efcind + d->island_efcadr[island];

    // sparse Jacobian
    if (mj_isSparse(m)) {
        for (int i = 0; i < resnnz; i++) {
            int row = resind[i];
            int Jnnz = d->efc_J_rownnz[row];
            int Jrowadr = d->efc_J_rowadr[row];
            int* Jind = d->efc_J_colind + Jrowadr;
            mjtNum* J = d->efc_J + Jrowadr;
            int j = flg_resunc ? row : i;
            res[j] = mju_dotSparse2(J, vec, Jnnz, Jind, vecnnz, vecind, flg_vecunc);
        }
    }

    // dense Jacobian
    else {
        int nv = m->nv;
        for (int i = 0; i < resnnz; i++) {
            int row = resind[i];
            int j = flg_resunc ? row : i;
            res[j] = mju_dotSparse(vec, d->efc_J + nv * row, vecnnz, vecind, flg_vecunc);
        }
    }
}



// multiply JacobianT by vector
void mj_mulJacTVec(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec) {
    // exit if no constraints
    if (!d->nefc) {
        return;
    }

    // sparse Jacobian
    if (mj_isSparse(m))
        mju_mulMatVecSparse(res, d->efc_JT, vec, m->nv,
            d->efc_JT_rownnz, d->efc_JT_rowadr,
            d->efc_JT_colind, d->efc_JT_rowsuper);

    // dense Jacobian
    else {
        mju_mulMatTVec(res, d->efc_J, vec, d->nefc, m->nv);
    }
}



// multiply Jacobian transpose by vector, for one island
//  flg_resunc and flg_vecunc denote whether res/vec are uncompressed
void mj_mulJacTVec_island(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec,
    int island, int flg_resunc, int flg_vecunc) {
    // no island, call regular function
    if (island < 0) {
        mj_mulJacTVec(m, d, res, vec);
        return;
    }

    // sizes
    int vecnnz = d->island_efcnum[island];
    int resnnz = d->island_dofnum[island];

    // indices
    int* vecind = d->island_efcind + d->island_efcadr[island];
    int* resind = d->island_dofind + d->island_dofadr[island];

    // sparse Jacobian
    if (mj_isSparse(m)) {
        for (int i = 0; i < resnnz; i++) {
            int row = resind[i];
            int JTnnz = d->efc_JT_rownnz[row];
            int JTrowadr = d->efc_JT_rowadr[row];
            int* JTind = d->efc_JT_colind + JTrowadr;
            mjtNum* JT = d->efc_JT + JTrowadr;
            int j = flg_resunc ? row : i;
            res[j] = mju_dotSparse2(JT, vec, JTnnz, JTind, vecnnz, vecind, flg_vecunc);
        }
    }

    // dense Jacobian
    else {
        int nefc = d->nefc;
        for (int i = 0; i < resnnz; i++) {
            int row = resind[i];
            int j = flg_resunc ? row : i;
            res[j] = mju_dotSparse(vec, d->efc_JT + nefc * row, vecnnz, vecind, flg_vecunc);
        }
    }
}


// compute efc_vel, efc_aref
void mj_referenceConstraint(const mjModel* m, mjData* d) {
    int nefc = d->nefc;
    mjtNum* KBIP = d->efc_KBIP;

    // compute efc_vel
    mj_mulJacVec(m, d, d->efc_vel, d->qvel);

    // compute aref = -B*vel - K*I*(pos-margin)
    for (int i = 0; i < nefc; i++) {
        d->efc_aref[i] = -KBIP[4 * i + 1] * d->efc_vel[i]
            - KBIP[4 * i] * KBIP[4 * i + 2] * (d->efc_pos[i] - d->efc_margin[i]);
    }
}