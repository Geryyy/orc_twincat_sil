#pragma once 

#include <stdint.h>
#include <mujoco_tc/mjmodel.h>


//-------------------------- name functions --------------------------------------------------------

// get string hash, see http://www.cse.yorku.ca/~oz/hash.html
uint64_t mj_hashString(const char* s, uint64_t n);

// get id of object with the specified mjtObj type and name, returns -1 if id not found
int mj_name2id(const mjModel* m, int type, const char* name);

// get name of object with the specified mjtObj type and id, returns NULL if name not found
const char* mj_id2name(const mjModel* m, int type, int id);