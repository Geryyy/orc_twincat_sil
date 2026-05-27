#pragma once

#include "TcInterfaces.h"
#include "TcRtInterfaces.h"
#include "mujoco_tc/mujoco_tc.h"

mjModel* mj_loadModelBuffer(const void* buffer, int buffer_sz, CTcTrace m_Trace);
mjtNum* mj_stackAllocNum(mjData* d, size_t size);
int safeAddToBufferSize(intptr_t* offset, size_t* nbuffer, size_t type_size, int nr, int nc);
void mj_makeRawData(mjData** dest, const mjModel* m);
mjData* mj_makeData(const mjModel* m);
int* mj_stackAllocInt(mjData* d, size_t size);
void* mj_stackAllocInfo(mjData* d, size_t bytes, size_t alignment, const char* caller, int line);

// mjData arena allocate
void* mj_arenaAllocByte(mjData* d, size_t bytes, size_t alignment);

// TC: Change __Alignof(type) to 1
// macro to allocate a stack array of given type, adds caller information
#define mjSTACKALLOC(d, num, type) \
(type*) mj_stackAllocInfo(d, (num) * sizeof(type), 1, __func__, __LINE__)


void mj_deleteModel(mjModel* m);
void mj_deleteData(mjData* d);

// mjData mark stack frame
void mj_markStack(mjData* d);

// mjData free stack frame
void mj_freeStack(mjData* d);

// clear arena pointers in mjData
static inline void mj_clearEfc(mjData* d) {
#define X(type, name, nr, nc) d->name = NULL;
  MJDATA_ARENA_POINTERS
#undef X
  d->nefc = 0;
  d->nisland = 0;
  d->contact = (mjContact*) d->arena;
}