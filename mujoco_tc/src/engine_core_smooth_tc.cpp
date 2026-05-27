#include "mujoco_tc/engine_macro.h"
#include "mujoco_tc/engine_util_blas_tc.h"
#include "mujoco_tc/engine_util_spatial_tc.h"
#include "mujoco_tc/engine_util_misc_tc.h"
#include "mujoco_tc/engine_io_tc.h"
#include "mujoco_tc/engine_crossplatform_tc.h"
#include "mujoco_tc/engine_support_tc.h"
#include "mujoco_tc/engine_core_smooth_tc.h"


// sparse L'*D*L factorizaton of inertia-like matrix M, assumed spd
// (legacy implementation)
void mj_factorI_legacy(const mjModel* m, mjData* d, const mjtNum* M, mjtNum* qLD,
    mjtNum* qLDiagInv) {
    int cnt;
    int Madr_kk, Madr_ki;
    mjtNum tmp;

    // local copies of key variables
    int* dof_Madr = m->dof_Madr;
    int* dof_parentid = m->dof_parentid;
    int nv = m->nv;

    // copy M into LD
    mju_copy(qLD, M, m->nM);

    // dense backward loop over dofs (regular only, simple diagonal already copied)
    for (int k=nv-1; k >= 0; k--) {
    // get address of M(k,k)
    Madr_kk = dof_Madr[k];

    // check for small/negative numbers on diagonal
    if (qLD[Madr_kk] < mjMINVAL) {
    mj_warning(d, mjWARN_INERTIA, k);
    qLD[Madr_kk] = mjMINVAL;
    }

    // skip the rest if simple
    if (m->dof_simplenum[k]) {
    continue;
}

    // sparse backward loop over ancestors of k (excluding k)
    Madr_ki = Madr_kk + 1;
    int i = dof_parentid[k];
    while (i >= 0) {
    tmp = qLD[Madr_ki] / qLD[Madr_kk];          // tmp = M(k,i) / M(k,k)

    // get number of ancestors of i (including i)
    if (i < nv-1) {
    cnt = dof_Madr[i+1] - dof_Madr[i];
    } else {
    cnt = m->nM - dof_Madr[i+1];
    }

    // M(i,j) -= M(k,j) * tmp
    mju_addToScl(qLD+dof_Madr[i], qLD+Madr_ki, -tmp, cnt);

    qLD[Madr_ki] = tmp;                         // M(k,i) = tmp

    // advance to i's parent
    i = dof_parentid[i];
    Madr_ki++;
    }
    }

    // compute 1/diag(D)
    for (int i=0; i < nv; i++) {
        qLDiagInv[i] = 1.0 / qLD[dof_Madr[i]];
    }
}

// sparse L'*D*L factorizaton of the inertia matrix M, assumed spd
void mj_factorM(const mjModel* m, mjData* d) {
    TM_START;
  
    // gather LD <- M (legacy to CSR) and factorize in-place
    mju_gather(d->qLD, d->qM, d->mapM2M, m->nM);
    mj_factorI(d->qLD, d->qLDiagInv, m->nv, d->M_rownnz, d->M_rowadr, m->dof_simplenum, d->M_colind);
  
    TM_ADD(mjTIMER_POS_INERTIA);
  }



// sparse L'*D*L factorizaton of inertia-like matrix M, assumed spd
void mj_factorI(mjtNum* mat, mjtNum* diaginv, int nv,
    const int* rownnz, const int* rowadr, const int* diagnum, const int* colind) {
    // backward loop over rows
    for (int k=nv-1; k >= 0; k--) {
    // get row k's address, diagonal index, inverse diagonal value
    int rowadr_k = rowadr[k];
    int diag_k = rowadr_k + rownnz[k] - 1;
    mjtNum invD = 1 / mat[diag_k];
    if (diaginv) diaginv[k] = invD;

    // skip if simple
    if (diagnum[k]) {
    continue;
    }

    // update triangle above row k, inclusive
    for (int adr=diag_k - 1; adr >= rowadr_k; adr--) {
    // tmp = L(k, i) / L(k, k)
    mjtNum tmp = mat[adr] * invD;

    // update row i < k:  L(i, 0..i) -= L(i, 0..i) * L(k, i) / L(k, k)
    int i = colind[adr];
    mju_addToScl(mat + rowadr[i], mat + rowadr_k, -tmp, rownnz[i]);

    // update ith element of row k:  L(k, i) /= L(k, k)
    mat[adr] = tmp;
    }
    }
}

// forward kinematics
void mj_kinematics(const mjModel* m, mjData* d) {
    int nbody = m->nbody, nsite = m->nsite, ngeom = m->ngeom;

    // set world position and orientation
    mju_zero3(d->xpos);
    mju_unit4(d->xquat);
    mju_zero3(d->xipos);
    mju_zero(d->xmat, 9);
    mju_zero(d->ximat, 9);
    d->xmat[0] = d->xmat[4] = d->xmat[8] = 1;
    d->ximat[0] = d->ximat[4] = d->ximat[8] = 1;

    // compute global cartesian positions and orientations of all bodies
    for (int i = 1; i < nbody; i++) {
        mjtNum xpos[3], xquat[4];
        int jntadr = m->body_jntadr[i];
        int jntnum = m->body_jntnum[i];

        // free joint
        if (jntnum == 1 && m->jnt_type[jntadr] == mjJNT_FREE) {
            // get qpos address
            int qadr = m->jnt_qposadr[jntadr];

            // copy pos and quat from qpos
            mju_copy3(xpos, d->qpos + qadr);
            mju_copy4(xquat, d->qpos + qadr + 3);
            mju_normalize4(xquat);

            // assign xanchor and xaxis
            mju_copy3(d->xanchor + 3 * jntadr, xpos);
            mju_copy3(d->xaxis + 3 * jntadr, m->jnt_axis + 3 * jntadr);
        }

        // regular or no joint
        else {
            int pid = m->body_parentid[i];

            // get body pos and quat: from model or mocap
            mjtNum* bodypos, * bodyquat, quat[4];
            if (m->body_mocapid[i] >= 0) {
                bodypos = d->mocap_pos + 3 * m->body_mocapid[i];
                mju_copy4(quat, d->mocap_quat + 4 * m->body_mocapid[i]);
                mju_normalize4(quat);
                bodyquat = quat;
            }
            else {
                bodypos = m->body_pos + 3 * i;
                bodyquat = m->body_quat + 4 * i;
            }

            // apply fixed translation and rotation relative to parent
            if (pid) {
                mju_mulMatVec3(xpos, d->xmat + 9 * pid, bodypos);
                mju_addTo3(xpos, d->xpos + 3 * pid);
                mju_mulQuat(xquat, d->xquat + 4 * pid, bodyquat);
            }
            else {
                // parent is the world
                mju_copy3(xpos, bodypos);
                mju_copy4(xquat, bodyquat);
            }

            // accumulate joints, compute xpos and xquat for this body
            mjtNum xanchor[3], xaxis[3];
            for (int j = 0; j < jntnum; j++) {
                // get joint id, qpos address, joint type
                int jid = jntadr + j;
                int qadr = m->jnt_qposadr[jid];
                mjtJoint jtype = (mjtJoint)(m->jnt_type[jid]);
                //mjtJoint jtype = m->jnt_type[jid];

                // compute axis in global frame; ball jnt_axis is (0,0,1), set by compiler
                mju_rotVecQuat(xaxis, m->jnt_axis + 3 * jid, xquat);

                // compute anchor in global frame
                mju_rotVecQuat(xanchor, m->jnt_pos + 3 * jid, xquat);
                mju_addTo3(xanchor, xpos);

                // apply joint transformation
                switch (jtype) {
                case mjJNT_SLIDE:
                    mju_addToScl3(xpos, xaxis, d->qpos[qadr] - m->qpos0[qadr]);
                    break;

                case mjJNT_BALL:
                case mjJNT_HINGE:
                {
                    // compute local quaternion rotation
                    mjtNum qloc[4];
                    if (jtype == mjJNT_BALL) {
                        mju_copy4(qloc, d->qpos + qadr);
                        mju_normalize4(qloc);
                    }
                    else {
                        mju_axisAngle2Quat(qloc, m->jnt_axis + 3 * jid, d->qpos[qadr] - m->qpos0[qadr]);
                    }

                    // apply rotation
                    mju_mulQuat(xquat, xquat, qloc);

                    // correct for off-center rotation
                    mjtNum vec[3];
                    mju_rotVecQuat(vec, m->jnt_pos + 3 * jid, xquat);
                    mju_sub3(xpos, xanchor, vec);
                }
                break;

                default:
                    mjERROR("unknown joint type %d", jtype);    // SHOULD NOT OCCUR
                }

                // assign xanchor and xaxis
                mju_copy3(d->xanchor + 3 * jid, xanchor);
                mju_copy3(d->xaxis + 3 * jid, xaxis);
            }
        }

        // assign xquat and xpos, construct xmat
        mju_normalize4(xquat);
        mju_copy4(d->xquat + 4 * i, xquat);
        mju_copy3(d->xpos + 3 * i, xpos);
        mju_quat2Mat(d->xmat + 9 * i, xquat);
    }

    // compute/copy Cartesian positions and orientations of body inertial frames
    for (int i = 1; i < nbody; i++) {
        mj_local2Global(d, d->xipos + 3 * i, d->ximat + 9 * i,
            m->body_ipos + 3 * i, m->body_iquat + 4 * i,
            i, m->body_sameframe[i]);
    }

    // compute/copy Cartesian positions and orientations of geoms
    for (int i = 0; i < ngeom; i++) {
        mj_local2Global(d, d->geom_xpos + 3 * i, d->geom_xmat + 9 * i,
            m->geom_pos + 3 * i, m->geom_quat + 4 * i,
            m->geom_bodyid[i], m->geom_sameframe[i]);
    }

    // compute/copy Cartesian positions and orientations of sites
    for (int i = 0; i < nsite; i++) {
        mj_local2Global(d, d->site_xpos + 3 * i, d->site_xmat + 9 * i,
            m->site_pos + 3 * i, m->site_quat + 4 * i,
            m->site_bodyid[i], m->site_sameframe[i]);
    }
}

// map inertias and motion dofs to global frame centered at subtree-CoM
void mj_comPos(const mjModel* m, mjData* d) {
    int nbody = m->nbody, njnt = m->njnt;
    mjtNum offset[3], axis[3];
    mj_markStack(d);
    mjtNum* mass_subtree = mjSTACKALLOC(d, m->nbody, mjtNum);

    // clear subtree
    mju_zero(mass_subtree, m->nbody);
    mju_zero(d->subtree_com, m->nbody * 3);

    // backwards pass over bodies: compute subtree_com and mass_subtree
    for (int i = nbody - 1; i >= 0; i--) {
        // add local info
        mju_addToScl3(d->subtree_com + 3 * i, d->xipos + 3 * i, m->body_mass[i]);
        mass_subtree[i] += m->body_mass[i];

        // add to parent, except for world
        if (i) {
            int j = m->body_parentid[i];
            mju_addTo3(d->subtree_com + 3 * j, d->subtree_com + 3 * i);
            mass_subtree[j] += mass_subtree[i];
        }

        // compute local com
        if (mass_subtree[i] < mjMINVAL) {
            mju_copy3(d->subtree_com + 3 * i, d->xipos + 3 * i);
        }
        else {
            mju_scl3(d->subtree_com + 3 * i, d->subtree_com + 3 * i,
                1.0 / mjMAX(mjMINVAL, mass_subtree[i]));
        }
    }

    // zero out CoM frame inertia for the world body
    mju_zero(d->cinert, 10);

    // map inertias to frame centered at subtree_com
    for (int i = 1; i < nbody; i++) {
        mju_sub3(offset, d->xipos + 3 * i, d->subtree_com + 3 * m->body_rootid[i]);
        mju_inertCom(d->cinert + 10 * i, m->body_inertia + 3 * i, d->ximat + 9 * i,
            offset, m->body_mass[i]);
    }

    // map motion dofs to global frame centered at subtree_com
    for (int j = 0; j < njnt; j++) {
        // get dof address, body index
        int da = 6 * m->jnt_dofadr[j];
        int bi = m->jnt_bodyid[j];

        // compute com-anchor vector
        mju_sub3(offset, d->subtree_com + 3 * m->body_rootid[bi], d->xanchor + 3 * j);

        // create motion dof
        int skip = 0;
        switch ((mjtJoint)m->jnt_type[j]) {
        case mjJNT_FREE:
            // translation components: x, y, z in global frame
            mju_zero(d->cdof + da, 18);
            for (int i = 0; i < 3; i++) {
                d->cdof[da + 3 + 7 * i] = 1;
            }

            // rotation components: same as ball
            skip = 18;
            mjFALLTHROUGH;

        case mjJNT_BALL:
            for (int i = 0; i < 3; i++) {
                // I_3 rotation in child frame (assume no subsequent rotations)
                axis[0] = d->xmat[9 * bi + i + 0];
                axis[1] = d->xmat[9 * bi + i + 3];
                axis[2] = d->xmat[9 * bi + i + 6];

                mju_dofCom(d->cdof + da + skip + 6 * i, axis, offset);
            }
            break;

        case mjJNT_SLIDE:
            mju_dofCom(d->cdof + da, d->xaxis + 3 * j, 0);
            break;

        case mjJNT_HINGE:
            mju_dofCom(d->cdof + da, d->xaxis + 3 * j, offset);
            break;
        }
    }

    mj_freeStack(d);
}




// composite rigid body inertia algorithm
void mj_crb(const mjModel* m, mjData* d) {
    TM_START;
    mjtNum buf[6];
    mjtNum* crb = d->crb;
    int last_body = m->nbody - 1, nv = m->nv;

    //crb = cinert
    mju_copy(crb, d->cinert, 10 * m->nbody);

    // backward pass over bodies, accumulate composite inertias
    for (int i = last_body; i > 0; i--) {
        if (m->body_parentid[i] > 0) {
            mju_addTo(crb + 10 * m->body_parentid[i], crb + 10 * i, 10);
        }
    }

    // clear qM
    mju_zero(d->qM, m->nM);

    // dense forward pass over dofs
    for (int i = 0; i < nv; i++) {
        // process block of diagonals (simple bodies)
        if (m->dof_simplenum[i]) {
            int n = i + m->dof_simplenum[i];
            for (; i < n; i++) {
                d->qM[m->dof_Madr[i]] = m->dof_M0[i];
            }

            // finish or else fall through with next row
            if (i == nv) {
                break;
            }
        }

        // init M(i,i) with armature inertia
        int Madr_ij = m->dof_Madr[i];
        d->qM[Madr_ij] = m->dof_armature[i];

        // precompute buf = crb_body_i * cdof_i
        mju_mulInertVec(buf, crb + 10 * m->dof_bodyid[i], d->cdof + 6 * i);

        // sparse backward pass over ancestors
        for (int j = i; j >= 0; j = m->dof_parentid[j]) {
            // M(i,j) += cdof_j * (crb_body_i * cdof_i)
            d->qM[Madr_ij++] += mju_dot(d->cdof + 6 * j, buf, 6);
        }
    }
    TM_END(mjTIMER_POS_INERTIA);
}


//---------------------------------- velocity ------------------------------------------------------

// compute cvel, cdof_dot
void mj_comVel(const mjModel* m, mjData* d) {
    int nbody = m->nbody;

    // set world vel to 0
    mju_zero(d->cvel, 6);

    // forward pass over bodies
    for (int i = 1; i < nbody; i++) {
        // get body's first dof address
        int bda = m->body_dofadr[i];

        // cvel = cvel_parent
        mjtNum cvel[6];
        mju_copy(cvel, d->cvel + 6 * m->body_parentid[i], 6);

        // cvel = cvel_parent + cdof * qvel,  cdofdot = cvel x cdof
        int dofnum = m->body_dofnum[i];
        mjtNum cdofdot[36];
        for (int j = 0; j < dofnum; j++) {
            mjtNum tmp[6];

            // compute cvel and cdofdot
            switch ((mjtJoint)m->jnt_type[m->dof_jntid[bda + j]]) {
            case mjJNT_FREE:
                // cdofdot = 0
                mju_zero(cdofdot, 18);

                // update velocity
                mju_mulDofVec(tmp, d->cdof + 6 * bda, d->qvel + bda, 3);
                mju_addTo(cvel, tmp, 6);

                // continue with rotations
                j += 3;
                mjFALLTHROUGH;

            case mjJNT_BALL:
                // compute all 3 cdofdots using parent velocity
                for (int k = 0; k < 3; k++) {
                    mju_crossMotion(cdofdot + 6 * (j + k), cvel, d->cdof + 6 * (bda + j + k));
                }

                // update velocity
                mju_mulDofVec(tmp, d->cdof + 6 * (bda + j), d->qvel + bda + j, 3);
                mju_addTo(cvel, tmp, 6);

                // adjust for 3-dof joint
                j += 2;
                break;

            default:
                // in principle we should use the new velocity to compute cdofdot,
                // but it makes no difference because crossMotion(cdof, cdof) = 0,
                // and using the old velocity may be more accurate numerically
                mju_crossMotion(cdofdot + 6 * j, cvel, d->cdof + 6 * (bda + j));

                // update velocity
                mju_mulDofVec(tmp, d->cdof + 6 * (bda + j), d->qvel + bda + j, 1);
                mju_addTo(cvel, tmp, 6);
            }
        }

        // assign cvel, cdofdot
        mju_copy(d->cvel + 6 * i, cvel, 6);
        mju_copy(d->cdof_dot + 6 * bda, cdofdot, 6*dofnum);
    }
}


//---------------------------------- RNE -----------------------------------------------------------

// RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term
void mj_rne(const mjModel* m, mjData* d, int flg_acc, mjtNum* result) {
    int nbody = m->nbody, nv = m->nv;
    mjtNum tmp[6], tmp1[6];
    mj_markStack(d);
    mjtNum* loc_cacc = mjSTACKALLOC(d, m->nbody * 6, mjtNum);
    mjtNum* loc_cfrc_body = mjSTACKALLOC(d, m->nbody * 6, mjtNum);

    // set world acceleration to -gravity
    mju_zero(loc_cacc, 6);
    if (!mjDISABLED(mjDSBL_GRAVITY)) {
        mju_scl3(loc_cacc + 3, m->opt.gravity, -1);
    }

    // forward pass over bodies: accumulate cacc, set cfrc_body
    for (int i = 1; i < nbody; i++) {
        // get body's first dof address
        int bda = m->body_dofadr[i];

        // cacc = cacc_parent + cdofdot * qvel
        mju_mulDofVec(tmp, d->cdof_dot + 6 * bda, d->qvel + bda, m->body_dofnum[i]);
        mju_add(loc_cacc + 6 * i, loc_cacc + 6 * m->body_parentid[i], tmp, 6);

        // cacc += cdof * qacc
        if (flg_acc) {
            mju_mulDofVec(tmp, d->cdof + 6 * bda, d->qacc + bda, m->body_dofnum[i]);
            mju_addTo(loc_cacc + 6 * i, tmp, 6);
        }

        // cfrc_body = cinert * cacc + cvel x (cinert * cvel)
        mju_mulInertVec(loc_cfrc_body + 6 * i, d->cinert + 10 * i, loc_cacc + 6 * i);
        mju_mulInertVec(tmp, d->cinert + 10 * i, d->cvel + 6 * i);
        mju_crossForce(tmp1, d->cvel + 6 * i, tmp);
        mju_addTo(loc_cfrc_body + 6 * i, tmp1, 6);
    }

    // clear world cfrc_body, for style
    mju_zero(loc_cfrc_body, 6);

    // backward pass over bodies: accumulate cfrc_body from children
    for (int i = nbody - 1; i > 0; i--)
        if (m->body_parentid[i]) {
            mju_addTo(loc_cfrc_body + 6 * m->body_parentid[i], loc_cfrc_body + 6 * i, 6);
        }

    // result = cdof * cfrc_body
    for (int i = 0; i < nv; i++) {
        result[i] = mju_dot(d->cdof + 6 * i, loc_cfrc_body + 6 * m->dof_bodyid[i], 6);
    }

    mj_freeStack(d);
}
