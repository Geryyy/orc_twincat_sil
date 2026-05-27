#include "mujoco_tc/engine_passive_tc.h"

#include <stddef.h>
#include <string.h>

#include "mujoco_tc/mjdata.h"
#include "mujoco_tc/mjmacro.h"
#include "mujoco_tc/mjmodel.h"
#include "mujoco_tc/engine_core_constraint_tc.h"
#include "mujoco_tc/engine_crossplatform_tc.h"
#include "mujoco_tc/engine_io_tc.h"
#include "mujoco_tc/engine_support_tc.h"
#include "mujoco_tc/engine_util_blas_tc.h"
#include "mujoco_tc/engine_util_misc_tc.h"
#include "mujoco_tc/engine_util_spatial_tc.h"

//----------------------------- passive forces -----------------------------------------------------

// local edge-based vertex indexing for 2D and 3D elements, 2D and 3D elements
// have 3 and 6 edges, respectively so the missing indexes are set to 0
static const int edges[2][6][2] = { {{1, 2}, {2, 0}, {0, 1}, {0, 0}, {0, 0}, {0, 0}},
                                   {{0, 1}, {1, 2}, {2, 0}, {2, 3}, {0, 3}, {1, 3}} };

// compute gradient of squared lengths of edges belonging to a given element
static void inline GradSquaredLengths(mjtNum gradient[6][2][3],
    const mjtNum* xpos,
    const int vert[4],
    const int edge[6][2],
    int nedge) {
    for (int e = 0; e < nedge; e++) {
        for (int d = 0; d < 3; d++) {
            gradient[e][0][d] = xpos[3 * vert[edge[e][0]] + d] - xpos[3 * vert[edge[e][1]] + d];
            gradient[e][1][d] = xpos[3 * vert[edge[e][1]] + d] - xpos[3 * vert[edge[e][0]] + d];
        }
    }
}

// TC: removed markStack and freeStack, removed major part of flex elasticity 
// spring and damper forces
static void mj_springdamper(const mjModel* m, mjData* d) {
    int nv = m->nv, njnt = m->njnt, ntendon = m->ntendon;
    int issparse = mj_isSparse(m);

    // joint-level springs
    for (int i = 0; i < njnt; i++) {
        mjtNum stiffness = m->jnt_stiffness[i];

        // disabled : nothing to do
        if (stiffness == 0) {
            continue;
        }

        int padr = m->jnt_qposadr[i];
        int dadr = m->jnt_dofadr[i];

        switch ((mjtJoint)m->jnt_type[i]) {
        case mjJNT_FREE:
            // apply force
            d->qfrc_spring[dadr + 0] = -stiffness * (d->qpos[padr + 0] - m->qpos_spring[padr + 0]);
            d->qfrc_spring[dadr + 1] = -stiffness * (d->qpos[padr + 1] - m->qpos_spring[padr + 1]);
            d->qfrc_spring[dadr + 2] = -stiffness * (d->qpos[padr + 2] - m->qpos_spring[padr + 2]);

            // continue with rotations
            dadr += 3;
            padr += 3;
            mjFALLTHROUGH;

        case mjJNT_BALL:
        {
            // convert quatertion difference into angular "velocity"
            mjtNum dif[3], quat[4];
            mju_copy4(quat, d->qpos + padr);
            mju_normalize4(quat);
            mju_subQuat(dif, quat, m->qpos_spring + padr);

            // apply torque
            d->qfrc_spring[dadr + 0] = -stiffness * dif[0];
            d->qfrc_spring[dadr + 1] = -stiffness * dif[1];
            d->qfrc_spring[dadr + 2] = -stiffness * dif[2];
        }
        break;

        case mjJNT_SLIDE:
        case mjJNT_HINGE:
            // apply force or torque
            d->qfrc_spring[dadr] = -stiffness * (d->qpos[padr] - m->qpos_spring[padr]);
            break;
        }
    }

    // dof-level dampers
    for (int i = 0; i < m->nv; i++) {
        mjtNum damping = m->dof_damping[i];
        if (damping != 0) {
            d->qfrc_damper[i] = -damping * d->qvel[i];
        }
    }

    // flex elasticity
    for (int f = 0; f < m->nflex; f++) {
        mjtNum* k = m->flex_stiffness + 21 * m->flex_elemadr[f];
        int dim = m->flex_dim[f];

        if (dim == 1 || m->flex_rigid[f] || k[0] == 0) {
            continue;
        }
        // if (m->flex_interp[f]) {
        //     mjtNum xpos[mjMAXFLEXNODES], displ[mjMAXFLEXNODES], vel[mjMAXFLEXNODES];
        //     mjtNum frc[mjMAXFLEXNODES], dmp[mjMAXFLEXNODES];
        //     mjtNum com[3] = {0};
        //     mjtNum* xpos0 = m->flex_node0 + 3*m->flex_nodeadr[f];
        //     int* bodyid = m->flex_nodebodyid + m->flex_nodeadr[f];
        //     int nstart = m->flex_nodeadr[f];
    
        //     // compute positions
        //     if (m->flex_centered[f]) {
        //     for (int i=0; i < m->flex_nodenum[f]; i++) {
        //         mju_copy3(xpos + 3*i, d->xpos + 3*bodyid[i]);
        //         mju_copy3(vel + 3*i, d->qvel + m->body_dofadr[bodyid[i]]);
        //     }
        //     } else {
        //     mjtNum screw[6];
        //     for (int i=0; i < m->flex_nodenum[f]; i++) {
        //         mju_mulMatVec3(xpos + 3*i, d->xmat + 9*bodyid[i], m->flex_node + 3*(i+nstart));
        //         mju_addTo3(xpos + 3*i, d->xpos + 3*bodyid[i]);
        //         mj_objectVelocity(m, d, mjOBJ_BODY, bodyid[i], screw, 0);
        //         mju_copy3(vel + 3*i, screw + 3);
        //     }
        //     }
    
        //     // compute center of mass
        //     for (int i = 0; i < m->flex_nodenum[f]; i++) {
        //     mju_addToScl3(com, xpos+3*i, 1.0/m->flex_nodenum[f]);
        //     }
    
        //     // re-center positions using center of mass
        //     for (int i = 0; i < m->flex_nodenum[f]; i++) {
        //     mju_addToScl3(xpos+3*i, com, -1);
        //     }
    
        //     // compute the Jacobian at the center of mass
        //     mjtNum mat[9] = {0};
        //     mjtNum p[3] = {.5, .5, .5};
        //     mju_defGradient(mat, p, xpos, 1);
    
        //     // find rotation
        //     mjtNum quat[4] = {1, 0, 0, 0};
        //     mju_mat2Rot(quat, mat);
        //     mju_negQuat(quat, quat);
    
        //     // rotate vertices to quat and add reference center of mass
        //     for (int i = 0; i < m->flex_nodenum[f]; i++) {
        //     mju_rotVecQuat(xpos+3*i, xpos+3*i, quat);
        //     mju_addTo3(xpos+3*i, p);
        //     mju_rotVecQuat(vel+3*i, vel+3*i, quat);
        //     }
    
        //     // compute displacement
        //     for (int i = 0; i < m->flex_nodenum[f]; i++) {
        //     mju_addScl3(displ+3*i, xpos+3*i, xpos0+3*i, -1);
        //     }
    
        //     // compute force in the stretch frame
        //     mju_mulMatVec(frc, k, displ, 3*m->flex_nodenum[f], 3*m->flex_nodenum[f]);
    
        //     // compute damping force in stretch frame
        //     mju_mulMatVec(dmp, k, vel, 3*m->flex_nodenum[f], 3*m->flex_nodenum[f]);
    
        //     // rotate forces to global frame and add to qfrc
        //     mju_negQuat(quat, quat);
        //     for (int i = 0; i < m->flex_nodenum[f]; i++) {
        //     mjtNum qfrc[3], qdmp[3];
        //     mju_rotVecQuat(qfrc, frc+3*i, quat);
        //     mju_rotVecQuat(qdmp, dmp+3*i, quat);
        //     mju_scl3(qdmp, qdmp, m->flex_damping[f]);
        //     if (m->flex_centered[f]) {
        //         mju_addTo3(d->qfrc_spring+m->body_dofadr[bodyid[i]], qfrc);
        //         mju_addTo3(d->qfrc_damper+m->body_dofadr[bodyid[i]], qdmp);
        //     } else {
        //         mj_applyFT(m, d, qfrc, 0, xpos+3*i, bodyid[i], d->qfrc_spring);
        //         mj_applyFT(m, d, qdmp, 0, xpos+3*i, bodyid[i], d->qfrc_damper);
        //     }
        //     }
    
        //     // do not continue with the rest of the flex passive forces
        //     continue;
        // }

        int nedge = (dim == 2) ? 3 : 6;
        int nvert = (dim == 2) ? 3 : 4;
        const int* elem = m->flex_elem + m->flex_elemdataadr[f];
        const int* edgeelem = m->flex_elemedge + m->flex_elemedgeadr[f];
        mjtNum* xpos = d->flexvert_xpos + 3 * m->flex_vertadr[f];
        mjtNum* vel = d->flexedge_velocity + m->flex_edgeadr[f];
        mjtNum* deformed = d->flexedge_length + m->flex_edgeadr[f];
        mjtNum* reference = m->flexedge_length0 + m->flex_edgeadr[f];
        int* bodyid = m->flex_vertbodyid + m->flex_vertadr[f];
        mjtNum kD = m->flex_damping[f] / m->opt.timestep;

        mj_markStack(d);
        mjtNum* qfrc = mjSTACKALLOC(d, 3 * m->flex_vertnum[f], mjtNum);
        mju_zero(qfrc, 3 * m->flex_vertnum[f]);

        // compute force element-by-element
        for (int t = 0; t < m->flex_elemnum[f]; t++) {
            const int* vert = elem + (dim + 1) * t;

            // compute length gradient with respect to dofs
            mjtNum gradient[6][2][3];
            GradSquaredLengths(gradient, xpos, vert, edges[dim - 2], nedge);

            // we add generalized Rayleigh damping as decribed in Section 5.2 of
            // Kharevych et al., "Geometric, Variational Integrators for Computer
            // Animation" http://multires.caltech.edu/pubs/DiscreteLagrangian.pdf

            // extract elongation of edges belonging to this element
            mjtNum elongation[6];
            for (int e = 0; e < nedge; e++) {
                int idx = edgeelem[t * nedge + e];
                mjtNum previous = deformed[idx] - vel[idx] * m->opt.timestep;
                elongation[e] = deformed[idx] * deformed[idx] - reference[idx] * reference[idx] +
                    (deformed[idx] * deformed[idx] - previous * previous) * kD;
            }

            // unpack triangular representation
            mjtNum metric[36];
            int id = 0;
            for (int ed1 = 0; ed1 < nedge; ed1++) {
                for (int ed2 = ed1; ed2 < nedge; ed2++) {
                    metric[nedge * ed1 + ed2] = k[21 * t + id];
                    metric[nedge * ed2 + ed1] = k[21 * t + id++];
                }
            }

            // we now multiply the elongations by the precomputed metric tensor,
            // notice that if metric=diag(1/reference) then this would yield a
            // mass-spring model

            // compute local force
            mjtNum force[12] = { 0 };
            for (int ed1 = 0; ed1 < nedge; ed1++) {
                for (int ed2 = 0; ed2 < nedge; ed2++) {
                    for (int i = 0; i < 2; i++) {
                        for (int x = 0; x < 3; x++) {
                            force[3 * edges[dim - 2][ed2][i] + x] -=
                                elongation[ed1] * gradient[ed2][i][x] *
                                metric[nedge * ed1 + ed2];
                        }
                    }
                }
            }

            // insert into global force
            for (int i = 0; i < nvert; i++) {
                for (int x = 0; x < 3; x++) {
                    qfrc[3 * vert[i] + x] += force[3 * i + x];
                }
            }
        }

        // insert force into qfrc_passive, straightforward for simple bodies,
        // need to distribute the force in case of pinned vertices
        for (int v = 0; v < m->flex_vertnum[f]; v++) {
            int bid = bodyid[v];
            if (m->body_simple[bid] != 2) {
                // this should only occur for pinned flex vertices
                mj_applyFT(m, d, qfrc + 3 * v, 0, xpos + 3 * v, bid, d->qfrc_spring);
            }
            else {
                int body_dofnum = m->body_dofnum[bid];
                int body_dofadr = m->body_dofadr[bid];
                for (int x = 0; x < body_dofnum; x++) {
                    d->qfrc_spring[body_dofadr + x] += qfrc[3 * v + x];
                }
            }
        }

        mj_freeStack(d);
    }

    // flexedge-level spring-dampers
    for (int f = 0; f < m->nflex; f++) {
        mjtNum stiffness = m->flex_edgestiffness[f];
        mjtNum damping = m->flex_edgedamping[f];

        // disabled or rigid: nothing to do
        if (m->flex_rigid[f] || (stiffness == 0 && damping == 0)) {
            continue;
        }

        // process non-rigid edges of this flex (global edge index)
        int edgeend = m->flex_edgeadr[f] + m->flex_edgenum[f];
        for (int e = m->flex_edgeadr[f]; e < edgeend; e++) {
            // skip rigid
            if (m->flexedge_rigid[e]) {
                continue;
            }

            // compute spring-damper force along edge
            mjtNum frc_spring = stiffness * (m->flexedge_length0[e] - d->flexedge_length[e]);
            mjtNum frc_damper = -damping * d->flexedge_velocity[e];

            // transform to joint torque, add to qfrc_{spring, damper}: dense or sparse
            if (issparse) {
                int end = d->flexedge_J_rowadr[e] + d->flexedge_J_rownnz[e];
                for (int j = d->flexedge_J_rowadr[e]; j < end; j++) {
                    int colind = d->flexedge_J_colind[j];
                    mjtNum J = d->flexedge_J[j];
                    d->qfrc_spring[colind] += J * frc_spring;
                    d->qfrc_damper[colind] += J * frc_damper;
                }
            }
            else {
                if (frc_spring) mju_addToScl(d->qfrc_spring, d->flexedge_J + e * nv, frc_spring, nv);
                if (frc_damper) mju_addToScl(d->qfrc_damper, d->flexedge_J + e * nv, frc_damper, nv);
            }
        }
    }

    // tendon-level spring-dampers
    for (int i = 0; i < ntendon; i++) {
        mjtNum stiffness = m->tendon_stiffness[i];
        mjtNum damping = m->tendon_damping[i];

        // disabled : nothing to do
        if (stiffness == 0 && damping == 0) {
            continue;
        }

        // compute spring force along tendon
        mjtNum length = d->ten_length[i];
        mjtNum lower = m->tendon_lengthspring[2 * i];
        mjtNum upper = m->tendon_lengthspring[2 * i + 1];
        mjtNum frc_spring = 0;
        if (length > upper) {
            frc_spring = stiffness * (upper - length);
        }
        else if (length < lower) {
            frc_spring = stiffness * (lower - length);
        }

        // compute damper linear force along tendon
        mjtNum frc_damper = -damping * d->ten_velocity[i];

        // transform to joint torque, add to qfrc_{spring, damper}: dense or sparse
        if (issparse) {
            if (frc_spring || frc_damper) {
                int end = d->ten_J_rowadr[i] + d->ten_J_rownnz[i];
                for (int j = d->ten_J_rowadr[i]; j < end; j++) {
                    int k = d->ten_J_colind[j];
                    mjtNum J = d->ten_J[j];
                    d->qfrc_spring[k] += J * frc_spring;
                    d->qfrc_damper[k] += J * frc_damper;
                }
            }
        }
        else {
            if (frc_spring) mju_addToScl(d->qfrc_spring, d->ten_J + i * nv, frc_spring, nv);
            if (frc_damper) mju_addToScl(d->qfrc_damper, d->ten_J + i * nv, frc_damper, nv);
        }
    }
}


// TC: Removed sections concerning fluids, gravcomp, plugins and calllbacks
// all passive forces
void mj_passive(const mjModel* m, mjData* d) {
    int nv = m->nv;

    // clear all passive force vectors
    mju_zero(d->qfrc_spring, nv);
    mju_zero(d->qfrc_damper, nv);
    mju_zero(d->qfrc_gravcomp, nv);
    mju_zero(d->qfrc_fluid, nv);
    mju_zero(d->qfrc_passive, nv);

    // disabled: return
    if (mjDISABLED(mjDSBL_PASSIVE)) {
        return;
    }

    // springs and dampers
    mj_springdamper(m, d);

    // gravity compensation
    int has_gravcomp = 0;
    //int has_gravcomp = mj_gravcomp(m, d);

    // fluid forces
    int has_fluid = 0;
    //int has_fluid = mj_fluid(m, d);

    // add passive forces into qfrc_passive
    mju_add(d->qfrc_passive, d->qfrc_spring, d->qfrc_damper, nv);
    if (has_fluid) mju_addTo(d->qfrc_passive, d->qfrc_fluid, nv);
    if (has_gravcomp) {
        int njnt = m->njnt;
        for (int i = 0; i < njnt; i++) {
            // skip if gravcomp added via actuators
            if (m->jnt_actgravcomp[i]) {
                continue;
            }

            // get number of dofs for this joint
            int dofnum;
            switch (m->jnt_type[i]) {
            case mjJNT_HINGE:
            case mjJNT_SLIDE:
                dofnum = 1;
                break;

            case mjJNT_BALL:
                dofnum = 3;
                break;

            case mjJNT_FREE:
                dofnum = 6;
                break;
            }

            // add gravcomp force
            int dofadr = m->jnt_dofadr[i];
            for (int j = 0; j < dofnum; j++) {
                d->qfrc_passive[dofadr + j] += d->qfrc_gravcomp[dofadr + j];
            }
        }
    }

    //// user callback: add custom passive forces
    //if (mjcb_passive) {
    //    mjcb_passive(m, d);
    //}

    //// plugin: add custom passive forces
    //if (m->nplugin) {
    //    const int nslot = mjp_pluginCount();

    //    // iterate over plugins, call compute if type is mjPLUGIN_PASSIVE
    //    for (int i = 0; i < m->nplugin; i++) {
    //        const int slot = m->plugin[i];
    //        const mjpPlugin* plugin = mjp_getPluginAtSlotUnsafe(slot, nslot);
    //        if (!plugin) {
    //            mjERROR("invalid plugin slot: %d", slot);
    //        }
    //        if (plugin->capabilityflags & mjPLUGIN_PASSIVE) {
    //            if (!plugin->compute) {
    //                mjERROR("`compute` is a null function pointer for plugin at slot %d", slot);
    //            }
    //            plugin->compute(m, d, i, mjPLUGIN_PASSIVE);
    //        }
    //    }
    //}
}
