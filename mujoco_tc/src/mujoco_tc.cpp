#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <type_traits>
#include <stdarg.h>

#include "TcMem.h"
#include "TcDef.h"
#include "TcInterfaces.h"
#include "TcRtInterfaces.h"


#include "orc/util/import_mujoco.h"
#include "mujoco_tc/mjmodel.h"
#include "mujoco_tc/mjplugin.h"
#include "mujoco_tc/mjxmacro.h"

#include "mujoco_tc/engine_io_tc.h"
#include "mujoco_tc/engine_util_blas_tc.h"
#include "mujoco_tc/engine_core_smooth_tc.h"

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define mjLOAD_MULTIPLE 2

// -------------------------------------------------------------------------------- errmem utils--------------------------------------------------------------------------------------------------------
//------------------------- default user handlers --------------------------------------------------


// define and clear handlers
void (*mju_user_error) (const char*) = 0;
void (*mju_user_warning) (const char*) = 0;
void* (*mju_user_malloc) (size_t) = 0;
void (*mju_user_free) (void*) = 0;


// restore default processing
void mju_clearHandlers(void) {
    mju_user_error = 0;
    mju_user_warning = 0;
    mju_user_malloc = 0;
    mju_user_free = 0;
}



static inline void* mju_alignedMalloc(size_t size, size_t align) {
#ifdef _WIN32
    //return _aligned_malloc(size, align); // TODOO
    return (void*)TcMemAllocate(size);
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
    return aligned_alloc(align, size);
#endif
}

static inline void mju_alignedFree(void* ptr) {
#ifdef _WIN32
    //_aligned_free(ptr); // TODOO
    //ptr = NULL;
    TcMemFree(ptr);
    ptr = NULL;
#else
    free(ptr);
#endif
}

typedef void (*callback_fn)(const char*);

static callback_fn _mjPRIVATE_tls_error_fn = NULL;
static callback_fn _mjPRIVATE_tls_warning_fn = NULL;

// override printf?
#define EXIT_FAILURE 1

void exit(int x) {
    // TODOO
    return;
}

static CTcTrace* trace_pointer;
//#undef mjERROR
//#define mjERROR(msg) trace_pointer->Log(tlVerbose, FENTERA msg);




// write datetime, type: message to MUJOCO_LOG.TXT
void mju_writeLog(const char* type, const char* msg) {
// TODOO
    return;
//    time_t rawtime;
//    struct tm timeinfo;
//    FILE* fp = fopen("MUJOCO_LOG.TXT", "a+t");
//    if (fp) {
//        // get time
//        time(&rawtime);
//
//#if defined(_POSIX_C_SOURCE) || defined(__APPLE__) || defined(__STDC_VERSION_TIME_H__)
//        localtime_r(&rawtime, &timeinfo);
//#elif _MSC_VER
//        localtime_s(&timeinfo, &rawtime);
//#elif __STDC_LIB_EXT1__
//        localtime_s(&rawtime, &timeinfo);
//#else
//#error "Thread-safe version of `localtime` is not present in the standard C library"
//#endif
//
//        // write to log file
//        fprintf(fp, "%s%s: %s\n\n", asctime(&timeinfo), type, msg);
//        fclose(fp);
//    }
}




void mju_error_raw(const char* msg) {
    if (_mjPRIVATE_tls_error_fn) {
        _mjPRIVATE_tls_error_fn(msg);
    }
    else if (mju_user_error) {
        mju_user_error(msg);
    }
    else {
        // write to log and console
        mju_writeLog("ERROR", msg);
        //printf("ERROR: %s\n\nPress Enter to exit ...", msg);

        // pause, exit
        //getchar();
        exit(EXIT_FAILURE);
    }
}

//------------------------------ malloc and free ---------------------------------------------------

// allocate memory; byte-align on 64; pad size to multiple of 64
void* mju_malloc(size_t size) {
    void* ptr = 0;

    // user allocator
    if (mju_user_malloc) {
        ptr = mju_user_malloc(size);
    }

    // default allocator
    else {
        // pad size to multiple of 64
        if ((size % 64)) {
            size += 64 - (size % 64);
        }

        // allocate
        ptr = mju_alignedMalloc(size, 64);
    }

    // error if null pointer
    if (!ptr) {
        mju_error("Could not allocate memory");
    }

    return ptr;
}


// free memory
void mju_free(void* ptr) {
    // return if null
    if (!ptr) {
        return;
    }

    // free with user or built-in function
    if (mju_user_free) {
        mju_user_free(ptr);
    }
    else {
        mju_alignedFree(ptr);
    }
}


// ------------------------------------------------------------------------------------ blas util -----------------------------------------------------------------------------------------------------
// res = 0
//void mju_zero(mjtNum* res, int n) {
//    memset(res, 0, n * sizeof(mjtNum));
//}
//
//
//// res = 0
//void mju_zero3(mjtNum res[3]) {
//    res[0] = 0;
//    res[1] = 0;
//    res[2] = 0;
//}



// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



// number of position and velocity coordinates for each joint type
const int nPOS[4] = { 7, 4, 1, 1 };
const int nVEL[4] = { 6, 3, 1, 1 };
















static int sensorSize(mjtSensor sensor_type, int sensor_dim) {
    switch (sensor_type) {
    case mjSENS_TOUCH:
    case mjSENS_RANGEFINDER:
    case mjSENS_JOINTPOS:
    case mjSENS_JOINTVEL:
    case mjSENS_TENDONPOS:
    case mjSENS_TENDONVEL:
    case mjSENS_ACTUATORPOS:
    case mjSENS_ACTUATORVEL:
    case mjSENS_ACTUATORFRC:
    case mjSENS_JOINTACTFRC:
    case mjSENS_JOINTLIMITPOS:
    case mjSENS_JOINTLIMITVEL:
    case mjSENS_JOINTLIMITFRC:
    case mjSENS_TENDONLIMITPOS:
    case mjSENS_TENDONLIMITVEL:
    case mjSENS_TENDONLIMITFRC:
    case mjSENS_GEOMDIST:
    case mjSENS_CLOCK:
        return 1;

    case mjSENS_CAMPROJECTION:
        return 2;

    case mjSENS_ACCELEROMETER:
    case mjSENS_VELOCIMETER:
    case mjSENS_GYRO:
    case mjSENS_FORCE:
    case mjSENS_TORQUE:
    case mjSENS_MAGNETOMETER:
    case mjSENS_BALLANGVEL:
    case mjSENS_FRAMEPOS:
    case mjSENS_FRAMEXAXIS:
    case mjSENS_FRAMEYAXIS:
    case mjSENS_FRAMEZAXIS:
    case mjSENS_FRAMELINVEL:
    case mjSENS_FRAMEANGVEL:
    case mjSENS_FRAMELINACC:
    case mjSENS_FRAMEANGACC:
    case mjSENS_SUBTREECOM:
    case mjSENS_SUBTREELINVEL:
    case mjSENS_SUBTREEANGMOM:
    case mjSENS_GEOMNORMAL:
        return 3;

    case mjSENS_GEOMFROMTO:
        return 6;

    case mjSENS_BALLQUAT:
    case mjSENS_FRAMEQUAT:
        return 4;

    case mjSENS_USER:
        return sensor_dim;

    case mjSENS_PLUGIN:
        return -1;

        // don't use a 'default' case, so compiler warns about missing values
    }
    return -1;
}


















//------------------------------- mjStatistic ------------------------------------------------------



// returns the number of objects of the given type
//   -1: mjOBJ_UNKNOWN
//   -2: invalid objtype
static int numObjects(const mjModel* m, mjtObj objtype) {
    switch (objtype) {
    case mjOBJ_FRAME:
    case mjOBJ_UNKNOWN:
        return -1;
    case mjOBJ_BODY:
    case mjOBJ_XBODY:
        return m->nbody;
    case mjOBJ_JOINT:
        return m->njnt;
    case mjOBJ_DOF:
        return m->nv;
    case mjOBJ_GEOM:
        return m->ngeom;
    case mjOBJ_SITE:
        return m->nsite;
    case mjOBJ_CAMERA:
        return m->ncam;
    case mjOBJ_LIGHT:
        return m->nlight;
    case mjOBJ_FLEX:
        return m->nflex;
    case mjOBJ_MESH:
        return m->nmesh;
    case mjOBJ_SKIN:
        return m->nskin;
    case mjOBJ_HFIELD:
        return m->nhfield;
    case mjOBJ_TEXTURE:
        return m->ntex;
    case mjOBJ_MATERIAL:
        return m->nmat;
    case mjOBJ_PAIR:
        return m->npair;
    case mjOBJ_EXCLUDE:
        return m->nexclude;
    case mjOBJ_EQUALITY:
        return m->neq;
    case mjOBJ_TENDON:
        return m->ntendon;
    case mjOBJ_ACTUATOR:
        return m->nu;
    case mjOBJ_SENSOR:
        return m->nsensor;
    case mjOBJ_NUMERIC:
        return m->nnumeric;
    case mjOBJ_TEXT:
        return m->ntext;
    case mjOBJ_TUPLE:
        return m->ntuple;
    case mjOBJ_KEY:
        return m->nkey;
    case mjOBJ_PLUGIN:
        return m->nplugin;
    case mjNOBJECT:
        return -2;
    }
    return -2;
}



/*
// validate reference fields in a model; return null if valid, error message otherwise
const char* mj_validateReferences(const mjModel* m) {
    // for each field in mjModel that refers to another field, call X with:
    //   adrarray: array containing the references
    //   nadrs:    number of elements in refarray
    //   ntarget:  number of elements in array where references are pointing
    //   numarray: if refarray is an adr array, numarray is the corresponding num array, otherwise 0

    // add flex fields (b/303056369)

#define MJMODEL_REFERENCES                                                         \
  X(body_parentid,      nbody,          nbody         , 0                      ) \
  X(body_rootid,        nbody,          nbody         , 0                      ) \
  X(body_weldid,        nbody,          nbody         , 0                      ) \
  X(body_mocapid,       nbody,          nmocap        , 0                      ) \
  X(body_jntadr,        nbody,          njnt          , m->body_jntnum         ) \
  X(body_dofadr,        nbody,          nv            , m->body_dofnum         ) \
  X(body_geomadr,       nbody,          ngeom         , m->body_geomnum        ) \
  X(body_bvhadr,        nbody,          nbvh          , m->body_bvhnum         ) \
  X(body_plugin,        nbody,          nplugin       , 0                      ) \
  X(jnt_qposadr,        njnt,           nq            , 0                      ) \
  X(jnt_dofadr,         njnt,           nv            , 0                      ) \
  X(jnt_bodyid,         njnt,           nbody         , 0                      ) \
  X(dof_bodyid,         nv,             nbody         , 0                      ) \
  X(dof_jntid,          nv,             njnt          , 0                      ) \
  X(dof_parentid,       nv,             nv            , 0                      ) \
  X(dof_Madr,           nv,             nM            , 0                      ) \
  X(geom_bodyid,        ngeom,          nbody         , 0                      ) \
  X(geom_matid,         ngeom,          nmat          , 0                      ) \
  X(site_bodyid,        nsite,          nbody         , 0                      ) \
  X(site_matid,         nsite,          nmat          , 0                      ) \
  X(cam_bodyid,         ncam,           nbody         , 0                      ) \
  X(cam_targetbodyid,   ncam,           nbody         , 0                      ) \
  X(light_bodyid,       nlight,         nbody         , 0                      ) \
  X(light_targetbodyid, nlight,         nbody         , 0                      ) \
  X(mesh_vertadr,       nmesh,          nmeshvert     , m->mesh_vertnum        ) \
  X(mesh_normaladr,     nmesh,          nmeshnormal   , m->mesh_normalnum      ) \
  X(mesh_texcoordadr,   nmesh,          nmeshtexcoord , m->mesh_texcoordnum    ) \
  X(mesh_faceadr,       nmesh,          nmeshface     , m->mesh_facenum        ) \
  X(mesh_bvhadr,        nmesh,          nbvh          , m->mesh_bvhnum         ) \
  X(mesh_graphadr,      nmesh,          nmeshgraph    , 0                      ) \
  X(flex_vertadr,       nflex,          nflexvert     , m->flex_vertnum        ) \
  X(flex_edgeadr,       nflex,          nflexedge     , m->flex_edgenum        ) \
  X(flex_elemadr,       nflex,          nflexelem     , m->flex_elemnum        ) \
  X(flex_evpairadr,     nflex,          nflexevpair   , m->flex_evpairnum      ) \
  X(flex_texcoordadr,   nflex,          nflextexcoord , 0                      ) \
  X(flex_elemdataadr,   nflex,          nflexelemdata , 0                      ) \
  X(flex_elemedgeadr,   nflex,          nflexelemedge , 0                      ) \
  X(flex_shelldataadr,  nflex,          nflexshelldata, 0                      ) \
  X(flex_edge,          nflexedge*2,    nflexvert     , 0                      ) \
  X(flex_elem,          nflexelemdata,  nflexvert     , 0                      ) \
  X(flex_elemedge,      nflexelemedge,  nflexedge     , 0                      ) \
  X(flex_shell,         nflexshelldata, nflexvert     , 0                      ) \
  X(flex_bvhadr,        nflex,          nbvh          , m->flex_bvhnum         ) \
  X(skin_matid,         nskin,          nmat          , 0                      ) \
  X(skin_vertadr,       nskin,          nskinvert     , m->skin_vertnum        ) \
  X(skin_texcoordadr,   nskin,          nskintexvert  , 0                      ) \
  X(skin_faceadr,       nskin,          nskinface     , m->skin_facenum        ) \
  X(skin_boneadr,       nskin,          nskinbone     , m->skin_bonenum        ) \
  X(skin_bonevertadr,   nskinbone,      nskinbonevert , m->skin_bonevertnum    ) \
  X(skin_bonebodyid,    nskinbone,      nbody         , 0                      ) \
  X(skin_bonevertid,    nskinbonevert,  nskinvert     , 0                      ) \
  X(pair_geom1,         npair,          ngeom         , 0                      ) \
  X(pair_geom2,         npair,          ngeom         , 0                      ) \
  X(actuator_plugin,    nu,             nplugin       , 0                      ) \
  X(actuator_actadr,    nu,             na            , m->actuator_actnum     ) \
  X(sensor_plugin,      nsensor,        nplugin       , 0                      ) \
  X(plugin_stateadr,    nplugin,        npluginstate  , m->plugin_statenum     ) \
  X(plugin_attradr,     nplugin,        npluginattr   , 0                      ) \
  X(tendon_adr,         ntendon,        nwrap         , m->tendon_num          ) \
  X(tendon_matid,       ntendon,        nmat          , 0                      ) \
  X(numeric_adr,        nnumeric,       nnumericdata  , m->numeric_size        ) \
  X(text_adr,           ntext,          ntextdata     , m->text_size           ) \
  X(tuple_adr,          ntuple,         ntupledata    , m->tuple_size          ) \
  X(name_bodyadr,       nbody,          nnames        , 0                      ) \
  X(name_jntadr,        njnt,           nnames        , 0                      ) \
  X(name_geomadr,       ngeom,          nnames        , 0                      ) \
  X(name_siteadr,       nsite,          nnames        , 0                      ) \
  X(name_camadr,        ncam,           nnames        , 0                      ) \
  X(name_lightadr,      nlight,         nnames        , 0                      ) \
  X(name_meshadr,       nmesh,          nnames        , 0                      ) \
  X(name_skinadr,       nskin,          nnames        , 0                      ) \
  X(name_hfieldadr,     nhfield,        nnames        , 0                      ) \
  X(name_texadr,        ntex,           nnames        , 0                      ) \
  X(name_matadr,        nmat,           nnames        , 0                      ) \
  X(name_pairadr,       npair,          nnames        , 0                      ) \
  X(name_excludeadr,    nexclude,       nnames        , 0                      ) \
  X(name_eqadr,         neq,            nnames        , 0                      ) \
  X(name_tendonadr,     ntendon,        nnames        , 0                      ) \
  X(name_actuatoradr,   nu,             nnames        , 0                      ) \
  X(name_sensoradr,     nsensor,        nnames        , 0                      ) \
  X(name_numericadr,    nnumeric,       nnames        , 0                      ) \
  X(name_textadr,       ntext,          nnames        , 0                      ) \
  X(name_tupleadr,      ntuple,         nnames        , 0                      ) \
  X(name_keyadr,        nkey,           nnames        , 0                      ) \
  X(hfield_pathadr,     nhfield,        npaths        , 0                      ) \
  X(mesh_pathadr,       nmesh,          npaths        , 0                      ) \
  X(skin_pathadr,       nskin,          npaths        , 0                      ) \
  X(tex_pathadr,        ntex,           npaths        , 0                      )

#define X(adrarray, nadrs, ntarget, numarray) {             \
    int *nums = (numarray);                                   \
    for (int i=0; i<m->nadrs; i++) {                          \
      int adrsmin = m->adrarray[i];                           \
      int num = (nums ? nums[i] : 1);                         \
      if (num < 0) {                                          \
        return "Invalid model: " #numarray " is negative.";   \
      }                                                       \
      if (num > MAX_ARRAY_SIZE) {                             \
        return "Invalid model: " #numarray " is too large.";  \
      }                                                       \
      int adrsmax = m->adrarray[i] + num;                     \
      if (adrsmax > m->ntarget || adrsmin < -1) {             \
        return "Invalid model: " #adrarray " out of bounds."; \
      }                                                       \
    }                                                         \
  }

    MJMODEL_REFERENCES;
#undef X
#undef MJMODEL_REFERENCES

    // special logic that doesn't fit in the macro:
    for (int i = 0; i < m->nbody; i++) {
        if (i > 0 && m->body_parentid[i] >= i) {
            return "Invalid model: bad body_parentid.";
        }
        if (m->body_rootid[i] > i) {
            return "Invalid model: bad body_rootid.";
        }
        if (m->body_weldid[i] > i) {
            return "Invalid model: bad body_weldid.";
        }
    }
    for (int i = 0; i < m->njnt; i++) {
        if (m->jnt_type[i] >= 4 || m->jnt_type[i] < 0) {
            return "Invalid model: jnt_type out of bounds.";
        }
        int jnt_qposadr = m->jnt_qposadr[i] + nPOS[m->jnt_type[i]];
        if (jnt_qposadr > m->nq || m->jnt_qposadr[i] < 0) {
            return "Invalid model: jnt_qposadr out of bounds.";
        }
        int jnt_dofadr = m->jnt_dofadr[i] + nVEL[m->jnt_type[i]];
        if (jnt_dofadr > m->nv || m->jnt_dofadr[i] < 0) {
            return "Invalid model: jnt_dofadr out of bounds.";
        }
    }
    for (int i = 0; i < m->nv; i++) {
        if (m->dof_parentid[i] >= i) {
            return "Invalid model: bad dof_parentid.";
        }
    }
    for (int i = 0; i < m->ngeom; i++) {
        if (m->geom_condim[i] > 6 || m->geom_condim[i] < 0) {
            return "Invalid model: geom_condim out of bounds.";
        }
        if (m->geom_type[i] == mjGEOM_HFIELD) {
            if (m->geom_dataid[i] >= m->nhfield || m->geom_dataid[i] < -1) {
                return "Invalid model: geom_dataid out of bounds.";
            }
        }
        else if ((m->geom_type[i] == mjGEOM_MESH) || (m->geom_type[i] == mjGEOM_SDF)) {
            if (m->geom_dataid[i] >= m->nmesh || m->geom_dataid[i] < -1) {
                return "Invalid model: geom_dataid out of bounds.";
            }
        }
    }
    for (int i = 0; i < m->nhfield; i++) {
        int hfield_adr = m->hfield_adr[i] + m->hfield_nrow[i] * m->hfield_ncol[i];
        if (hfield_adr > m->nhfielddata || m->hfield_adr[i] < 0) {
            return "Invalid model: hfield_adr out of bounds.";
        }
    }
    for (int i = 0; i < m->ntex; i++) {
        int tex_adr = m->tex_adr[i] + 3 * m->tex_height[i] * m->tex_width[i];
        if (tex_adr > m->ntexdata || m->tex_adr[i] < 0) {
            return "Invalid model: tex_adr out of bounds.";
        }
    }
    for (int i = 0; i < m->npair; i++) {
        int pair_body1 = (m->pair_signature[i] & 0xFFFF);
        if (pair_body1 >= m->nbody || pair_body1 < 0) {
            return "Invalid model: pair_body1 out of bounds.";
        }
        int pair_body2 = (m->pair_signature[i] >> 16);
        if (pair_body2 >= m->nbody || pair_body2 < 0) {
            return "Invalid model: pair_body2 out of bounds.";
        }
    }
    for (int i = 0; i < m->neq; i++) {
        int obj1id = m->eq_obj1id[i];
        int obj2id = m->eq_obj2id[i];
        int objtype = m->eq_objtype[i];
        switch ((mjtEq)m->eq_type[i]) {
        case mjEQ_JOINT:
            if (obj1id >= m->njnt || obj1id < 0) {
                return "Invalid model: eq_obj1id out of bounds.";
            }
            // -1 is the value used if second object is omitted.
            if (obj2id >= m->njnt || obj2id < -1) {
                return "Invalid model: eq_obj2id out of bounds.";
            }
            break;

        case mjEQ_TENDON:
            if (obj1id >= m->ntendon || obj1id < 0) {
                return "Invalid model: eq_obj1id out of bounds.";
            }
            // -1 is the value used if second object is omitted.
            if (obj2id >= m->ntendon || obj2id < -1) {
                return "Invalid model: eq_obj2id out of bounds.";
            }
            break;

        case mjEQ_WELD:
        case mjEQ_CONNECT:
            if (objtype == mjOBJ_BODY) {
                if (obj1id >= m->nbody || obj1id < 0) {
                    return "Invalid model: eq_obj1id out of bounds.";
                }
                if (obj2id >= m->nbody || obj2id < 0) {
                    return "Invalid model: eq_obj2id out of bounds.";
                }
            }
            else if (objtype == mjOBJ_SITE) {
                if (obj1id >= m->nsite || obj1id < 0) {
                    return "Invalid model: eq_obj1id out of bounds.";
                }
                if (obj2id >= m->nsite || obj2id < 0) {
                    return "Invalid model: eq_obj2id out of bounds.";
                }
            }
            else {
                return "Invalid model: eq_objtype is not body or site.";
            }
            break;

        case mjEQ_FLEX:
            if (obj1id >= m->nflex || obj1id < 0) {
                return "Invalid model: eq_obj1id out of bounds.";
            }

            // -1 is the value used if second object is omitted
            if (obj2id != -1) {
                return "Invalid model: eq_obj2id must be -1.";
            }
            break;

        default:
            // might occur in case of the now-removed distance equality constraint
            mjERROR("unknown equality constraint type.");
        }
    }
    for (int i = 0; i < m->nwrap; i++) {
        int wrap_objid = m->wrap_objid[i];
        switch ((mjtWrap)m->wrap_type[i]) {
        case mjWRAP_NONE:
        case mjWRAP_PULLEY:
            // wrap_objid not used.
            break;
        case mjWRAP_JOINT:
            if (wrap_objid >= m->njnt || wrap_objid < 0) {
                return "Invalid model: wrap_objid out of bounds.";
            }
            break;
        case mjWRAP_SITE:
            if (wrap_objid >= m->nsite || wrap_objid < 0) {
                return "Invalid model: wrap_objid out of bounds.";
            }
            break;
        case mjWRAP_SPHERE:
        case mjWRAP_CYLINDER:
            if (wrap_objid >= m->ngeom || wrap_objid < 0) {
                return "Invalid model: wrap_objid out of bounds.";
            }
            break;
        }
    }
    for (int i = 0; i < m->nu; i++) {
        int actuator_trntype = m->actuator_trntype[i];
        int id = m->actuator_trnid[2 * i];
        int idslider = m->actuator_trnid[2 * i + 1];
        switch ((mjtTrn)actuator_trntype) {
        case mjTRN_JOINT:
        case mjTRN_JOINTINPARENT:
            if (id < 0 || id >= m->njnt) {
                return "Invalid model: actuator_trnid out of bounds.";
            }
            break;
        case mjTRN_TENDON:
            if (id < 0 || id >= m->ntendon) {
                return "Invalid model: actuator_trnid out of bounds.";
            }
            break;
        case mjTRN_SITE:
            if (id < 0 || id >= m->nsite) {
                return "Invalid model: actuator_trnid out of bounds.";
            }
            break;
        case mjTRN_SLIDERCRANK:
            if (id < 0 || id >= m->nsite) {
                return "Invalid model: actuator_trnid out of bounds.";
            }
            if (idslider < 0 || idslider >= m->nsite) {
                return "Invalid model: actuator_trnid out of bounds.";
            }
            break;
        case mjTRN_BODY:
            if (id < 0 || id >= m->nbody) {
                return "Invalid model: actuator_trnid out of bounds.";
            }
            break;
        case mjTRN_UNDEFINED:
            // actuator_trnid not used.
            break;
        }
    }
    for (int i = 0; i < m->nsensor; i++) {
        mjtSensor sensor_type = m->sensor_type[i];
        int sensor_size;
        if (sensor_type == mjSENS_PLUGIN) {
            const mjpPlugin* plugin = mjp_getPluginAtSlot(m->plugin[m->sensor_plugin[i]]);
            if (!plugin->nsensordata) {
                mjERROR("`nsensordata` is a null function pointer for plugin at slot %d",
                    m->plugin[m->sensor_plugin[i]]);
            }
            sensor_size = plugin->nsensordata(m, m->sensor_plugin[i], i);
        }
        else {
            sensor_size = sensorSize(sensor_type, m->sensor_dim[i]);
        }
        if (sensor_size < 0) {
            return "Invalid model: Bad sensor_type.";
        }
        int sensor_adr = m->sensor_adr[i];
        if (sensor_adr < 0 || sensor_adr + sensor_size > m->nsensordata) {
            return "Invalid model: sensor_adr out of bounds.";
        }
        int nobj = numObjects(m, m->sensor_objtype[i]);
        if (nobj == -2) {
            return "Invalid model: invalid sensor_objtype";
        }
        if (nobj != -1 && (m->sensor_objid[i] < 0 || m->sensor_objid[i] >= nobj)) {
            return "Invalid model: invalid sensor_objid";
        }
        nobj = numObjects(m, m->sensor_reftype[i]);
        if (nobj == -2) {
            return "Invalid model: invalid sensor_reftype";
        }
        if (nobj != -1 && (m->sensor_refid[i] < -1 || m->sensor_refid[i] >= nobj)) {
            return "Invalid model: invalid sensor_refid";
        }
    }
    for (int i = 0; i < m->nexclude; i++) {
        int exclude_body1 = (m->exclude_signature[i] & 0xFFFF);
        if (exclude_body1 >= m->nbody || exclude_body1 < 0) {
            return "Invalid model: exclude_body1 out of bounds.";
        }
        int exclude_body2 = (m->exclude_signature[i] >> 16);
        if (exclude_body2 >= m->nbody || exclude_body2 < 0) {
            return "Invalid model: exclude_body2 out of bounds.";
        }
    }
    for (int i = 0; i < m->ntuple; i++) {
        for (int j = 0; j < m->tuple_size[i]; j++) {
            int adr = m->tuple_adr[i] + j;
            int nobj = numObjects(m, m->tuple_objtype[adr]);
            if (nobj == -2) {
                return "Invalid model: invalid tuple_objtype";
            }
            if (nobj != -1 && (m->tuple_objid[adr] < 0 || m->tuple_objid[adr] >= nobj)) {
                return "Invalid model: invalid tuple_objid";
            }
        }
    }

    return NULL;
}
*/


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
