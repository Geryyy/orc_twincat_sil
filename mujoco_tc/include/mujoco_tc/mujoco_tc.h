#pragma once
#ifdef TC_VER



#include "mujoco_tc/mjmodel.h"
#include "mujoco_tc/mjdata.h"
#include "mujoco_tc/mjmacro.h"
#include "mujoco_tc/mjxmacro.h"
#include "mujoco_tc/engine_name_tc.h"

#include "mujoco_tc/engine_io_tc.h"
#include "mujoco_tc/engine_util_spatial_tc.h"
#include "mujoco_tc/engine_util_blas_tc.h"

#include "TcInterfaces.h"
#include "TcRtInterfaces.h"

#define mjERROR(...)                                                          \
{                                                                             \
  char _errbuf[1024];                                                         \
  size_t _funclen = strlen(__func__);                                         \
  strncpy(_errbuf, __func__, sizeof(_errbuf));                                \
  snprintf(_errbuf + _funclen, sizeof(_errbuf) - _funclen, ": " __VA_ARGS__); \
  mju_error_raw(_errbuf);                                                     \
}

#define mjERRORTC(trace, msg) trace.Log(tlVerbose, FENTERA msg)

#define mjLOAD_MULTIPLE 2

//void mju_zero(mjtNum* res, int n);
//void mju_zero3(mjtNum res[3]);
void mju_error_raw(const char* msg);
void mju_error(const char* msg, ...);
void mju_free(void* ptr);
void* mju_malloc(size_t size);
void mju_warning(const char* msg, ...);

// engine_forward_tc.h
void mj_fwdPosition(const mjModel* m, mjData* d);
void mj_fwdVelocity(const mjModel* m, mjData* d);
void mj_forward(const mjModel* m, mjData* d);

// engine_support_tc.h

// convert sparse inertia matrix M into full matrix
void mj_fullM(const mjModel* m, mjtNum* dst, const mjtNum* M);


// compute 3/6-by-nv Jacobian of global point attached to given body
void mj_jac(const mjModel* m, const mjData* d,
    mjtNum* jacp, mjtNum* jacr, const mjtNum point[3], int body);

// compute body frame Jacobian
void mj_jacBody(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int body);
// compute site Jacobian
void mj_jacSite(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int site);

void mju_printMat(const mjtNum* mat, int nr, int nc, CTcTrace m_Trace);
void mju_printVec(const mjtNum* vec, int n, CTcTrace m_Trace);

// integrate qpos with given qvel
void mj_integratePos(const mjModel* m, mjtNum* qpos, const mjtNum* qvel, mjtNum dt);

#else
	#include <mujoco/mujoco.h>
#endif 