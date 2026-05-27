#pragma once

#include "mujoco_tc/mjmodel.h"
#include "mujoco_tc/mjdata.h"

//-------------------------- Jacobian-related ------------------------------------------------------

// determine type of friction cone
int mj_isPyramidal(const mjModel* m);

// determine type of constraint Jacobian
int mj_isSparse(const mjModel* m);

// determine type of solver
int mj_isDual(const mjModel* m);

// multiply Jacobian by vector
void mj_mulJacVec(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);

// multiply Jacobian by vector, for one island
void mj_mulJacVec_island(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec,
    int island, int flg_resunc, int flg_vecunc);

// multiply JacobianT by vector
void mj_mulJacTVec(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);

// multiply JacobianT by vector, for one island
void mj_mulJacTVec_island(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec,
    int island, int flg_resunc, int flg_vecunc);

//-------------------------- utility functions -----------------------------------------------------

// assign/override solver reference parameters
void mj_assignRef(const mjModel* m, mjtNum* target, const mjtNum* source);

// assign/override solver impedance parameters
void mj_assignImp(const mjModel* m, mjtNum* target, const mjtNum* source);

// assign/clamp contact friction parameters
void mj_assignFriction(const mjModel* m, mjtNum* target, const mjtNum* source);

// assign/override geom margin
mjtNum mj_assignMargin(const mjModel* m, mjtNum source);

// add contact to d->contact list; return 0 if success; 1 if buffer full
int mj_addContact(const mjModel* m, mjData* d, const mjContact* con);

//


void mj_referenceConstraint(const mjModel* m, mjData* d);