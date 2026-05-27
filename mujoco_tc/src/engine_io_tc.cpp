#include <type_traits>

#include "TcDef.h"
#include "TcBase.h"
#include "TcError.h"
#include "OsBase.h"
#include "TcInterfaces.h"
#include "TcRtInterfaces.h"

#include "mujoco_tc/mjsan.h"
#include "mujoco_tc/engine_io_tc.h"
#include "mujoco_tc/mjmodel.h"
#include "mujoco_tc/mjplugin.h"
#include "mujoco_tc/mjdata.h"
#include "mujoco_tc/mjxmacro.h"
#include "mujoco_tc/engine_macro.h"
#include "mujoco_tc/mujoco_tc.h"
#include "mujoco_tc/engine_crossplatform_tc.h"
#include "mujoco_tc/engine_util_misc_tc.h"
#include "mujoco_tc/engine_util_blas_tc.h"
#include "mujoco_tc/thread_pool_tc.h"

// id used to identify binary mjModel file/buffer
static const int ID = 54321;

// number of ints in the mjb header
#define NHEADER 5

// macro for referring to a mjModel member in generic expressions
// ARC: additional parentheses removed as this led to problems
#define MJMODEL_MEMBER(name) ((mjModel*) NULL)->name

static const int MAX_ARRAY_SIZE = INT_MAX / 4;

// add red zone padding when built with asan, to detect out-of-bound accesses
#ifdef ADDRESS_SANITIZER
#define mjREDZONE 32
#else
#define mjREDZONE 0
#endif

typedef struct {
    size_t pbase;   // value of d->pbase immediately before mj_markStack
    size_t pstack;  // value of d->pstack immediately before mj_markStack
    void* pc;       // program counter of the call site of mj_markStack (only set when under asan)
} mjStackFrame;

// compute a % b with a fast code path if the second argument is a power of 2
static inline size_t fastmod(size_t a, size_t b) {
    // (b & (b - 1)) == 0 implies that b is a power of 2
    if (mjLIKELY((b & (b - 1)) == 0)) {
        return a & (b - 1);
    }
    return a % b;
}


void mju_error(const char* msg, ...) {
    // TODOO
    return;
}
// write message to logfile and console
void mju_warning(CTcTrace m_Trace, const char* msg, ...) {
    m_Trace.Log(tlVerbose, "Mujoco warning %s", msg);
    return;
}
void mju_warning(const char* msg, ...) {
    //m_Trace.Log(tlVerbose, "Mujoco warning %s", msg);
    return;
}


// free model memory without destroying the struct
static void freeModelBuffers(mjModel* m) {
    mju_free(m->buffer);
}

// number of bytes to be skipped to achieve 64-byte alignment
static inline unsigned int SKIP(intptr_t offset) {
    const unsigned int align = 64;
    // compute skipped bytes
    return (align - (offset % align)) % align;
}

// set pointers in mjModel buffer
static void mj_setPtrModel(mjModel* m) {
    char* ptr = (char*)m->buffer;

    // prepare symbols needed by xmacro
    MJMODEL_POINTERS_PREAMBLE(m);

    // assign pointers with padding
#define X(type, name, nr, nc)                             \
  m->name = (type*)(ptr + SKIP((intptr_t)ptr));           \
  ASAN_POISON_MEMORY_REGION(ptr, PTRDIFF(m->name, ptr));  \
  ptr += SKIP((intptr_t)ptr) + sizeof(type)*(m->nr)*(nc);

    MJMODEL_POINTERS
#undef X

        // check size
        ptrdiff_t sz = ptr - (char*)m->buffer;
    if (m->nbuffer != sz) {
        mjERROR("mjModel buffer size mismatch, expected size: %zd,  actual size: %zu", m->nbuffer, sz);
    }
}

// read from memory buffer
static void bufread(void* dest, int num, int szbuf, const void* buf, int* ptrbuf) {
    // check pointers
    if (!dest || !buf || !ptrbuf) {
        mjERROR("NULL pointer passed to bufread");
    }

    // check size
    if (*ptrbuf + num > szbuf) {
        mjERROR("attempting to read outside model buffer");
    }

    // read, advance pointer
    memcpy(dest, (char*)buf + *ptrbuf, num);
    *ptrbuf += num;
}


// count ints in mjModel
static int getnint(void) {
    int cnt = 0;
#define X(name) if(std::is_same<decltype(MJMODEL_MEMBER(name)), int>::value) cnt += 1;
    //#define X(name) cnt += _Generic(MJMODEL_MEMBER(name), int: 1, default: 0);
    MJMODEL_INTS
#undef X

        return cnt;
}

// count size_t members in mjModel
static int getnsize(void) {
    int cnt = 0;

#define X(name) if(std::is_same<decltype(MJMODEL_MEMBER(name)), size_t>::value)  cnt += 1;
    //#define X(name) cnt += _Generic(MJMODEL_MEMBER(name), size_t: 1, default: 0);
    MJMODEL_INTS
#undef X

        return cnt;
}

// count pointers in mjModel
static int getnptr(void) {
    int cnt = 0;

#define X(type, name, nr, nc) cnt++;
    MJMODEL_POINTERS
#undef X

        return cnt;
}


// set statistics to default values; compute later in compiler
void mj_defaultStatistic(mjStatistic* stat) {
    mju_zero3(stat->center);
    stat->extent = 2;
    stat->meaninertia = 1;
    stat->meanmass = 1;
    stat->meansize = 0.2;
}


//------------------------------- mjVisual ---------------------------------------------------------

// set 4 floats
static void setf4(float* rgba, float r, float g, float b, float a) {
    rgba[0] = r;
    rgba[1] = g;
    rgba[2] = b;
    rgba[3] = a;
}


// set visual options to default values
// TC: Added F ad the ending of magic numbers, such that no warning for truncation arises
void mj_defaultVisual(mjVisual* vis) {
    // global
    vis->global.orthographic = 0;
    vis->global.fovy = 45;
    vis->global.ipd = 0.068F;
    vis->global.azimuth = 90;
    vis->global.elevation = -45;
    vis->global.linewidth = 1.0;
    vis->global.glow = 0.3F;
    vis->global.offwidth = 640;
    vis->global.offheight = 480;
    vis->global.realtime = 1.0;
    vis->global.ellipsoidinertia = 0;
    vis->global.bvactive = 1;

    // rendering quality
    vis->quality.shadowsize = 4096;
    vis->quality.offsamples = 4;
    vis->quality.numslices = 28;
    vis->quality.numstacks = 16;
    vis->quality.numquads = 4;

    // head light
    vis->headlight.ambient[0] = 0.1F;
    vis->headlight.ambient[1] = 0.1F;
    vis->headlight.ambient[2] = 0.1F;
    vis->headlight.diffuse[0] = 0.4F;
    vis->headlight.diffuse[1] = 0.4F;
    vis->headlight.diffuse[2] = 0.4F;
    vis->headlight.specular[0] = 0.5;
    vis->headlight.specular[1] = 0.5;
    vis->headlight.specular[2] = 0.5;
    vis->headlight.active = 1;

    // map parameters
    vis->map.stiffness = 100;
    vis->map.stiffnessrot = 500;
    vis->map.force = 0.005F;
    vis->map.torque = 0.1F;
    vis->map.alpha = 0.3F;
    vis->map.fogstart = 3.0;
    vis->map.fogend = 10.0;
    vis->map.znear = 0.01F;
    vis->map.zfar = 50.0;
    vis->map.haze = 0.3F;
    vis->map.shadowclip = 1.0;
    vis->map.shadowscale = 0.6F;
    vis->map.actuatortendon = 2.0;

    // size parameters
    vis->scale.forcewidth = 0.1F;
    vis->scale.contactwidth = 0.3F;
    vis->scale.contactheight = 0.1F;
    vis->scale.connect = 0.2F;
    vis->scale.com = 0.4F;
    vis->scale.camera = 0.3F;
    vis->scale.light = 0.3F;
    vis->scale.selectpoint = 0.2F;
    vis->scale.jointlength = 1.0;
    vis->scale.jointwidth = 0.1F;
    vis->scale.actuatorlength = 0.7F;
    vis->scale.actuatorwidth = 0.2F;
    vis->scale.framelength = 1.0;
    vis->scale.framewidth = 0.1F;
    vis->scale.constraint = 0.1F;
    vis->scale.slidercrank = 0.2F;
    vis->scale.frustum = 10.0;

    // colors
    setf4(vis->rgba.fog, 0.F, 0.F, 0.F, 1.);
    setf4(vis->rgba.haze, 1.F, 1.F, 1.F, 1.);
    setf4(vis->rgba.force, 1.F, .5F, .5F, 1.);
    setf4(vis->rgba.inertia, .8F, .2F, .2F, .6F);
    setf4(vis->rgba.joint, .2F, .6F, .8F, 1.F);
    setf4(vis->rgba.actuator, .2F, .25F, .2F, 1);
    setf4(vis->rgba.actuatornegative, .2F, .6F, .9F, 1.F);
    setf4(vis->rgba.actuatorpositive, .9F, .4F, .2F, 1.F);
    setf4(vis->rgba.com, .9F, .9F, .9F, 1.F);
    setf4(vis->rgba.camera, .6F, .9F, .6F, 1.F);
    setf4(vis->rgba.light, .6F, .6F, .9F, 1.F);
    setf4(vis->rgba.selectpoint, .9F, .9F, .1F, 1.F);
    setf4(vis->rgba.connect, .2F, .2F, .8F, 1.F);
    setf4(vis->rgba.contactpoint, .9F, .6F, .2F, 1.F);
    setf4(vis->rgba.contactforce, .7F, .9F, .9F, 1.F);
    setf4(vis->rgba.contactfriction, .9F, .8F, .4F, 1.F);
    setf4(vis->rgba.contacttorque, .9F, .7F, .9F, 1.F);
    setf4(vis->rgba.contactgap, .5F, .8F, .9F, 1.F);
    setf4(vis->rgba.rangefinder, 1.F, 1.F, .1F, 1.F);
    setf4(vis->rgba.constraint, .9F, .0F, .0F, 1.F);
    setf4(vis->rgba.slidercrank, .5F, .3F, .8F, 1.F);
    setf4(vis->rgba.crankbroken, .9F, .0F, .0F, 1.F);
    setf4(vis->rgba.frustum, 1.F, 1.F, .0F, .2F);
    setf4(vis->rgba.bv, 0.F, 1.F, .0F, .5);
    setf4(vis->rgba.bvactive, 1.F, 0.F, .0F, .5);
}


//------------------------------- mjOption ---------------------------------------------------------

// set default solver parameters
void mj_defaultSolRefImp(mjtNum* solref, mjtNum* solimp) {
    if (solref) {
        solref[0] = 0.02;       // timeconst
        solref[1] = 1;          // dampratio
    }

    if (solimp) {
        solimp[0] = 0.9;        // dmin
        solimp[1] = 0.95;       // dmax
        solimp[2] = 0.001;      // width
        solimp[3] = 0.5;        // midpoint
        solimp[4] = 2;          // power
    }
}



// set model options to default values
void mj_defaultOption(mjOption* opt) {
    // fill opt with zeros in case struct is padded
    memset(opt, 0, sizeof(mjOption));

    // timing parameters
    opt->timestep = 0.002;
    opt->apirate = 100;

    // solver parameters
    opt->impratio = 1;
    opt->tolerance = 1e-8;
    opt->ls_tolerance = 0.01;
    opt->noslip_tolerance = 1e-6;
    opt->ccd_tolerance = 1e-6;

    // physical constants
    opt->gravity[0] = 0;
    opt->gravity[1] = 0;
    opt->gravity[2] = -9.81;
    opt->wind[0] = 0;
    opt->wind[1] = 0;
    opt->wind[2] = 0;
    opt->magnetic[0] = 0;
    opt->magnetic[1] = -0.5;
    opt->magnetic[2] = 0;
    opt->density = 0;
    opt->viscosity = 0;

    // solver overrides
    opt->o_margin = 0;
    mj_defaultSolRefImp(opt->o_solref, opt->o_solimp);
    opt->o_friction[0] = 1;
    opt->o_friction[1] = 1;
    opt->o_friction[2] = 0.005;
    opt->o_friction[3] = 0.0001;
    opt->o_friction[4] = 0.0001;

    // discrete options
    opt->integrator = mjINT_EULER;
    opt->cone = mjCONE_PYRAMIDAL;
    opt->jacobian = mjJAC_AUTO;
    opt->solver = mjSOL_NEWTON;
    opt->iterations = 100;
    opt->ls_iterations = 50;
    opt->noslip_iterations = 0;
    opt->ccd_iterations = 50;
    opt->disableflags = 0;
    opt->enableflags = 0;
    opt->disableactuator = 0;

    // sdf collisions
    opt->sdf_initpoints = 40;
    opt->sdf_iterations = 10;
}


//-------------------------- sparse system matrix construction -------------------------------------

// construct sparse representation of dof-dof matrix
static void makeDofDofSparse(const mjModel* m, mjData* d,
    int* rownnz, int* rowadr,  int* diag, int* colind,
    int reduced, int upper) {
int nv = m->nv;

// no dofs, nothing to do
if (!nv) {
return;
}

mj_markStack(d);
int* remaining = mjSTACKALLOC(d, nv, int);

// compute rownnz
mju_zeroInt(rownnz, nv);
for (int i = nv - 1; i >= 0; i--) {
// init at diagonal
int j = i;
rownnz[i]++;

// process below diagonal unless reduced and dof is simple
if (!(reduced && m->dof_simplenum[i])) {
while ((j = m->dof_parentid[j]) >= 0) {
// both reduced and non-reduced have lower triangle
rownnz[i]++;

// add upper triangle if requested
if (upper) rownnz[j]++;
}
}
}

// accumulate rowadr
rowadr[0] = 0;
for (int i = 1; i < nv; i++) {
rowadr[i] = rowadr[i - 1] + rownnz[i - 1];
}

// populate colind
mju_copyInt(remaining, rownnz, nv);
for (int i = nv - 1; i >= 0; i--) {
// init at diagonal
remaining[i]--;
colind[rowadr[i] + remaining[i]] = i;

// process below diagonal unless reduced and dof is simple
if (!(reduced && m->dof_simplenum[i])) {
int j = i;
while ((j = m->dof_parentid[j]) >= 0) {
remaining[i]--;
colind[rowadr[i] + remaining[i]] = j;

// add upper triangle if requested
if (upper) {
remaining[j]--;
colind[rowadr[j] + remaining[j]] = i;
}
}
}
}

// check for remaining; SHOULD NOT OCCUR
for (int i = 0; i < nv; i++) {
if (remaining[i] != 0) {
mjERROR("unexpected remaining");
}
}

// check total nnz; SHOULD NOT OCCUR
int expected_nnz = upper ? m->nD : (reduced ? m->nC : m->nM);
if (rowadr[nv - 1] + rownnz[nv - 1] != expected_nnz) {
mjERROR("sum of rownnz different from expected");
}

// find diagonal indices
if (diag) {
for (int i = 0; i < nv; i++) {
int adr = rowadr[i];
int j = 0;
while (colind[adr + j] < i && j < rownnz[i]) {
j++;
}
if (colind[adr + j] != i) {
mjERROR("diagonal index not found");
}
diag[i] = j;
}
}

mj_freeStack(d);
}

// construct sparse representation of body-dof matrix
static void makeBSparse(const mjModel* m, mjData* d) {
int nv = m->nv, nbody = m->nbody;
int* rownnz = d->B_rownnz;
int* rowadr = d->B_rowadr;
int* colind = d->B_colind;

// set rownnz to subtree dofs counts, including self
mju_zeroInt(rownnz, nbody);
for (int i = nbody - 1; i > 0; i--) {
rownnz[i] += m->body_dofnum[i];
rownnz[m->body_parentid[i]] += rownnz[i];
}

// check if rownnz[0] != nv; SHOULD NOT OCCUR
if (rownnz[0] != nv) {
mjERROR("rownnz[0] different from nv");
}

// add dofs in ancestors bodies
for (int i = 0; i < nbody; i++) {
int j = m->body_parentid[i];
while (j > 0) {
rownnz[i] += m->body_dofnum[j];
j = m->body_parentid[j];
}
}

// compute rowadr
rowadr[0] = 0;
for (int i = 1; i < nbody; i++) {
rowadr[i] = rowadr[i - 1] + rownnz[i - 1];
}

// check if total nnz != nB; SHOULD NOT OCCUR
if (m->nB != rowadr[nbody - 1] + rownnz[nbody - 1]) {
mjERROR("sum of rownnz different from nB");
}

// allocate and clear incremental row counts
mj_markStack(d);
int* cnt = mjSTACKALLOC(d, nbody, int);
mju_zeroInt(cnt, nbody);

// add subtree dofs to colind
for (int i = nbody - 1; i > 0; i--) {
// add this body's dofs to subtree
for (int n = 0; n < m->body_dofnum[i]; n++) {
colind[rowadr[i] + cnt[i]] = m->body_dofadr[i] + n;
cnt[i]++;
}

// add body subtree to parent
int par = m->body_parentid[i];
for (int n = 0; n < cnt[i]; n++) {
colind[rowadr[par] + cnt[par]] = colind[rowadr[i] + n];
cnt[par]++;
}
}

// add all ancestor dofs
for (int i = 0; i < nbody; i++) {
int par = m->body_parentid[i];
while (par > 0) {
// add ancestor body dofs
for (int n = 0; n < m->body_dofnum[par]; n++) {
colind[rowadr[i] + cnt[i]] = m->body_dofadr[par] + n;
cnt[i]++;
}

// advance to parent
par = m->body_parentid[par];
}
}

// process all bodies
for (int i = 0; i < nbody; i++) {
// make sure cnt = rownnz; SHOULD NOT OCCUR
if (rownnz[i] != cnt[i]) {
mjERROR("cnt different from rownnz");
}

// sort colind in each row
if (cnt[i] > 1) {
mju_insertionSortInt(colind + rowadr[i], cnt[i]);
}
}

mj_freeStack(d);
}



// check D and B sparsity for consistency
static void checkDBSparse(const mjModel* m, mjData* d) {
// process all dofs
for (int j = 0; j < m->nv; j++) {
// get body for this dof
int i = m->dof_bodyid[j];

// D[row j] and B[row i] should be identical
if (d->D_rownnz[j] != d->B_rownnz[i]) {
mjERROR("rows have different nnz");
}
for (int k = 0; k < d->D_rownnz[j]; k++) {
if (d->D_colind[d->D_rowadr[j] + k] != d->B_colind[d->B_rowadr[i] + k]) {
mjERROR("rows have different colind");
}
}
}
}



// integer valued dst[D or C or M] = src[M (legacy)], handle different sparsity representations
static void copyM2Sparse(const mjModel* m, mjData* d, int* dst, const int* src,
int reduced, int upper) {
int nv = m->nv;
const int* rownnz;
const int* rowadr;
if (reduced && !upper) {
rownnz = d->C_rownnz;
rowadr = d->C_rowadr;
} else if (!reduced && !upper) {
rownnz = d->M_rownnz;
rowadr = d->M_rowadr;
} else if (!reduced && upper) {
rownnz = d->D_rownnz;
rowadr = d->D_rowadr;
} else {
mjERROR("unsupported sparsity structure (reduced + upper)");
}

mj_markStack(d);

// init remaining
int* remaining = mjSTACKALLOC(d, nv, int);
mju_copyInt(remaining, rownnz, nv);

// copy data
for (int i = nv - 1; i >= 0; i--) {
// init at diagonal
int adr = m->dof_Madr[i];
remaining[i]--;
dst[rowadr[i] + remaining[i]] = src[adr];
adr++;

// process below diagonal unless reduced and dof is simple
if (!(reduced && m->dof_simplenum[i])) {
int j = i;
while ((j = m->dof_parentid[j]) >= 0) {
remaining[i]--;
dst[rowadr[i] + remaining[i]] = src[adr];

// add upper triangle if requested
if (upper) {
remaining[j]--;
dst[rowadr[j] + remaining[j]] = src[adr];
}

adr++;
}
}
}

// check that none remaining
for (int i=0; i < nv; i++) {
if (remaining[i]) {
mjERROR("unassigned index");
}
}

mj_freeStack(d);
}



// integer valued dst[M] = src[D lower]
static void copyD2MSparse(const mjModel* m, const mjData* d, int* dst, const int* src) {
int nv = m->nv;

// copy data
for (int i = nv - 1; i >= 0; i--) {
// find diagonal in qDeriv
int j = 0;
while (d->D_colind[d->D_rowadr[i] + j] < i) {
j++;
}

// copy
int adr = m->dof_Madr[i];
while (j >= 0) {
dst[adr] = src[d->D_rowadr[i] + j];
adr++;
j--;
}
}
}



// construct index mappings between M <-> D, M -> C, M (legacy) -> M (CSR)
static void makeDofDofmaps(const mjModel* m, mjData* d) {
int nM = m->nM, nC = m->nC, nD = m->nD;
mj_markStack(d);

// make mapM2D
int* M = mjSTACKALLOC(d, nM, int);
for (int i=0; i < nM; i++) M[i] = i;
for (int i=0; i < nD; i++) d->mapM2D[i] = -1;
copyM2Sparse(m, d, d->mapM2D, M, /*reduced=*/0, /*upper=*/1);

// check that all indices are filled in
for (int i=0; i < nD; i++) {
if (d->mapM2D[i] < 0) {
mjERROR("unassigned index in mapM2D");
}
}

// make mapD2M
int* D = mjSTACKALLOC(d, nD, int);
for (int i=0; i < nD; i++) D[i] = i;
for (int i=0; i < nM; i++) d->mapD2M[i] = -1;
copyD2MSparse(m, d, d->mapD2M, D);

// check that all indices are filled in
for (int i=0; i < nM; i++) {
if (d->mapD2M[i] < 0) {
mjERROR("unassigned index in mapD2M");
}
}

// make mapM2C
for (int i=0; i < nC; i++) d->mapM2C[i] = -1;
copyM2Sparse(m, d, d->mapM2C, M, /*reduced=*/1, /*upper=*/0);

// check that all indices are filled in
for (int i=0; i < nC; i++) {
if (d->mapM2C[i] < 0) {
mjERROR("unassigned index in mapM2C");
}
}

// make mapM2M
for (int i=0; i < nM; i++) d->mapM2M[i] = -1;
copyM2Sparse(m, d, d->mapM2M, M, /*reduced=*/0, /*upper=*/0);

// check that all indices are filled in
for (int i=0; i < nM; i++) {
if (d->mapM2M[i] < 0) {
mjERROR("unassigned index in mapM2M");
}
}

mj_freeStack(d);
}




/////////


// increases buffer size without causing integer overflow, returns 0 if
// operation would cause overflow
// performs the following operations:
// *nbuffer += SKIP(*offset) + type_size*nr*nc;
// *offset += SKIP(*offset) + type_size*nr*nc;
static int safeAddToBufferSize(intptr_t* offset, size_t* nbuffer,
    size_t type_size, int nr, int nc) {
    if (type_size < 0 || nr < 0 || nc < 0) {
        return 0;
    }
    // TODO: offer a safe implementation for MSVC or other compilers that don't have the builtins
    *nbuffer += SKIP(*offset) + type_size * nr * nc;
    *offset += SKIP(*offset) + type_size * nr * nc;

    return 1;
}

// clear data, set defaults
// TC: Removed all plugin usage
static void _resetData(const mjModel* m, mjData* d, unsigned char debug_value) {
    //------------------------------ save plugin state and data
    //mjtNum* plugin_state;
    //uintptr_t* plugindata;
    //if (d->nplugin) {
    //    plugin_state = mju_malloc(sizeof(mjtNum) * m->npluginstate);
    //    memcpy(plugin_state, d->plugin_state, sizeof(mjtNum) * m->npluginstate);
    //    plugindata = mju_malloc(sizeof(uintptr_t) * m->nplugin);
    //    memcpy(plugindata, d->plugin_data, sizeof(uintptr_t) * m->nplugin);
    //}

    //------------------------------ clear header

    // clear stack pointer
    if (!d->threadpool) {
        d->pstack = 0;
    }
    d->pbase = 0;

    // clear arena pointers
    d->parena = 0;

    // poison the entire arena+stack memory region when built with asan
#ifdef ADDRESS_SANITIZER
    ASAN_POISON_MEMORY_REGION(d->arena, d->narena);
#endif

#ifdef MEMORY_SANITIZER
    __msan_allocated_memory(d->arena, d->narena);
#endif

#define X(type, name, nr, nc) d->name = NULL;
    MJDATA_ARENA_POINTERS
#undef X
    d->contact = (mjContact *)d->arena;
    //d->contact = d->arena;

    // clear memory utilization stats
    d->maxuse_stack = 0;
    mju_zeroSizeT(d->maxuse_threadstack, mjMAXTHREAD);
    d->maxuse_arena = 0;
    d->maxuse_con = 0;
    d->maxuse_efc = 0;

    // clear solver diagnostics
    memset(d->warning, 0, mjNWARNING * sizeof(mjWarningStat));
    memset(d->timer, 0, mjNTIMER * sizeof(mjTimerStat));
    memset(d->solver, 0, mjNSOLVER * mjNISLAND * sizeof(mjSolverStat));
    mju_zeroInt(d->solver_niter, mjNISLAND);
    mju_zeroInt(d->solver_nnz, mjNISLAND);
    mju_zero(d->solver_fwdinv, 2);

    // clear variable sizes
    d->ncon = 0;
    d->ne = 0;
    d->nf = 0;
    d->nl = 0;
    d->nefc = 0;
    d->nJ = 0;
    d->nA = 0;
    d->nisland = 0;

    // clear global properties
    d->time = 0;
    mju_zero(d->energy, 2);

    //------------------------------ clear buffer, set defaults

    // fill buffer with debug_value (normally 0)
#ifdef ADDRESS_SANITIZER
    {
#define X(type, name, nr, nc) memset(d->name, (int)debug_value, sizeof(type)*(m->nr)*(nc));
        MJDATA_POINTERS_PREAMBLE(m)
            MJDATA_POINTERS
#undef X
    }
#else
    memset(d->buffer, (int)debug_value, d->nbuffer);
#endif

#ifdef MEMORY_SANITIZER
    // Tell msan to treat the entire buffer as uninitialized
    __msan_allocated_memory(d->buffer, d->nbuffer);
#endif

    // zero out arrays that are not affected by mj_forward
    mju_zero(d->qpos, m->nq);
    mju_zero(d->qvel, m->nv);
    mju_zero(d->act, m->na);
    mju_zero(d->ctrl, m->nu);
    for (int i = 0; i < m->neq; i++) d->eq_active[i] = m->eq_active0[i];
    mju_zero(d->qfrc_applied, m->nv);
    mju_zero(d->xfrc_applied, 6 * m->nbody);
    mju_zero(d->qacc, m->nv);
    mju_zero(d->qacc_warmstart, m->nv);
    mju_zero(d->act_dot, m->na);
    mju_zero(d->userdata, m->nuserdata);
    mju_zero(d->sensordata, m->nsensordata);
    mju_zero(d->mocap_pos, 3 * m->nmocap);
    mju_zero(d->mocap_quat, 4 * m->nmocap);

    // copy qpos0 from model
    if (m->qpos0) {
        memcpy(d->qpos, m->qpos0, m->nq * sizeof(mjtNum));
    }

  // set mocap_pos/quat = body_pos/quat for mocap bodies
  if (m->body_mocapid) {
    for (int i=0; i < m->nbody; i++) {
      int id = m->body_mocapid[i];
      if (id >= 0) {
        mju_copy3(d->mocap_pos+3*id, m->body_pos+3*i);
        mju_copy4(d->mocap_quat+4*id, m->body_quat+4*i);
      }
    }
  } else {
    // set the mocap_quats to {1, 0, 0, 0}
    for (int i=0; i < m->nmocap; i++) {
      d->mocap_quat[4*i] = 1.0;
    }
  }

  // construct sparse matrix representations
  if (m->body_dofadr) {
    // make D
    makeDofDofSparse(m, d, d->D_rownnz, d->D_rowadr, d->D_diag, d->D_colind,
                     /*reduced=*/0, /*upper=*/1);

    // make B, check D and B
    makeBSparse(m, d);
    checkDBSparse(m, d);

    // make M, C
    makeDofDofSparse(m, d, d->M_rownnz, d->M_rowadr, NULL, d->M_colind, /*reduced=*/0, /*upper=*/0);
    makeDofDofSparse(m, d, d->C_rownnz, d->C_rowadr, NULL, d->C_colind, /*reduced=*/1, /*upper=*/0);

    // make index mappings: mapM2D, mapD2M, mapM2C, mapM2M
    makeDofDofmaps(m, d);
  }

//   // restore pluginstate and plugindata
//   if (d->nplugin) {
//     memcpy(d->plugin_state, plugin_state, sizeof(mjtNum) * m->npluginstate);
//     mju_free(plugin_state);
//     memcpy(d->plugin_data, plugindata, sizeof(uintptr_t) * m->nplugin);
//     mju_free(plugindata);

//     // restore the plugin array back into d and reset the instances
//     for (int i = 0; i < m->nplugin; ++i) {
//       d->plugin[i] = m->plugin[i];
//       const mjpPlugin* plugin = mjp_getPluginAtSlot(m->plugin[i]);
//       if (plugin->reset) {
//         plugin->reset(m, &d->plugin_state[m->plugin_stateadr[i]],
//                       (void*)(d->plugin_data[i]), i);
//       }
//     }
//   }

  // copy signature from model
  d->signature = m->signature;
}


// free mjData memory without destroying the struct
// TC: Removed plugin deletion
static void freeDataBuffers(mjData* d) {
#ifdef ADDRESS_SANITIZER
    // raise an error if there's a dangling stack frame
    mj_freeStack(d);
#endif

    // destroy plugin instances
    //for (int i = 0; i < d->nplugin; ++i) {
    //    const mjpPlugin* plugin = mjp_getPluginAtSlot(d->plugin[i]);
    //    if (plugin->destroy) {
    //        plugin->destroy(d, i);
    //    }
    //}
    mju_free(d->buffer);
    mju_free(d->arena);
}

// clear data, set data->qpos = model->qpos0
void mj_resetData(const mjModel* m, mjData* d) {
    _resetData(m, d, 0);
}


// set pointers into mjData buffer
static void mj_setPtrData(const mjModel* m, mjData* d) {
    char* ptr = (char*)d->buffer;

    // prepare symbols needed by xmacro
    MJDATA_POINTERS_PREAMBLE(m);

    // assign pointers with padding
#define X(type, name, nr, nc)                             \
  d->name = (type*)(ptr + SKIP((intptr_t)ptr));           \
  ASAN_POISON_MEMORY_REGION(ptr, PTRDIFF(d->name, ptr));  \
  ptr += SKIP((intptr_t)ptr) + sizeof(type)*(m->nr)*(nc);

    MJDATA_POINTERS
#undef X

        // check size
        ptrdiff_t sz = ptr - (char*)d->buffer;
    if (d->nbuffer != sz) {
        mjERROR("mjData buffer size mismatch, expected size: %zd,  actual size: %zu", d->nbuffer, sz);
    }

    // zero-initialize arena pointers
#define X(type, name, nr, nc) d->name = NULL;
    MJDATA_ARENA_POINTERS
#undef X

        //d->contact = d->arena;
        d->contact = (mjContact*)(d->arena);
}


// allocate and initialize raw mjData structure
void mj_makeRawData(mjData** dest, const mjModel* m) {
    intptr_t offset = 0;
    int allocate = *dest ? 0 : 1;
    mjData* d = NULL;

    // allocate mjData
    if (!allocate) {
        d = *dest;
        freeDataBuffers(d);
    }
    else {
        d = (mjData*)mju_malloc(sizeof(mjData));
    }

    if (!d) {
        mjERROR("could not allocate mjData");
    }



    // prepare symbols needed by xmacro
    MJDATA_POINTERS_PREAMBLE(m);

    // compute buffer size
    d->nbuffer = 0;
    d->buffer = d->arena = NULL;
#define X(type, name, nr, nc)                                                \
  if (!safeAddToBufferSize(&offset, &d->nbuffer, sizeof(type), m->nr, nc)) { \
    if (allocate) mju_free(d);                                               \
    mju_warning("Invalid data: " #name " too large.");                       \
    return;                                                                  \
  }

    MJDATA_POINTERS
#undef X

        // copy stack size from model
        d->narena = m->narena;

    // allocate buffer
    d->buffer = mju_malloc(d->nbuffer);
    if (!d->buffer) {
        if (allocate) mju_free(d);
        mjERROR("could not allocate mjData buffer");
    }

    // allocate arena
    d->arena = mju_malloc(d->narena);
    if (!d->arena) {
        mju_free(d->buffer);
        if (allocate) mju_free(d);
        mjERROR("could not allocate mjData arena");
    }

    // set pointers into buffer
    mj_setPtrData(m, d);

    // clear threadpool
    d->threadpool = 0;

    // clear nplugin (overwritten by _initPlugin)
    d->nplugin = 0;

    // copy pointer if allocated here
    if (allocate) {
        *dest = d;
    }
   
}

// allocate and initialize mjData structure
// TC: Removed plugin initialization
mjData* mj_makeData(const mjModel* m) {
    mjData* d = NULL;
    mj_makeRawData(&d, m);
    if (d) {
        //mj_initPlugin(m, d);
        mj_resetData(m, d);
    }
    return d;
}

//////////////////////////////////////


// allocate and initialize mjModel structure
void mj_makeModel(mjModel** dest,
    int nq, int nv, int nu, int na, int nbody, int nbvh,
    int nbvhstatic, int nbvhdynamic, int njnt, int ngeom, int nsite, int ncam,
    int nlight, int nflex, int nflexnode, int nflexvert, int nflexedge, int nflexelem,
    int nflexelemdata, int nflexelemedge, int nflexshelldata, int nflexevpair, int nflextexcoord,
    int nmesh, int nmeshvert, int nmeshnormal, int nmeshtexcoord, int nmeshface,
    int nmeshgraph, int nmeshpoly, int nmeshpolyvert, int nmeshpolymap, int nskin, int nskinvert,
    int nskintexvert, int nskinface,
    int nskinbone, int nskinbonevert, int nhfield, int nhfielddata, int ntex,
    int ntexdata, int nmat, int npair, int nexclude, int neq, int ntendon,
    int nwrap, int nsensor, int nnumeric, int nnumericdata, int ntext,
    int ntextdata, int ntuple, int ntupledata, int nkey, int nmocap,
    int nplugin, int npluginattr, int nuser_body, int nuser_jnt, int nuser_geom,
    int nuser_site, int nuser_cam, int nuser_tendon, int nuser_actuator,
    int nuser_sensor, int nnames, int npaths) {
  intptr_t offset = 0;
  int allocate = *dest ? 0 : 1;
  mjModel* m = NULL;

  // allocate mjModel
  if (!allocate) {
    m = *dest;
    freeModelBuffers(m);
  } else {
    m = (mjModel*)mju_malloc(sizeof(mjModel));
  }

  if (!m) {
    mjERROR("could not allocate mjModel");
  }
  memset(m, 0, sizeof(mjModel));

  // set size parameters
  m->nq = nq;
  m->nv = nv;
  m->nu = nu;
  m->na = na;
  m->nbody = nbody;
  m->nbvh = nbvh;
  m->nbvhstatic = nbvhstatic;
  m->nbvhdynamic = nbvhdynamic;
  m->njnt = njnt;
  m->ngeom = ngeom;
  m->nsite = nsite;
  m->ncam = ncam;
  m->nlight = nlight;
  m->nflex = nflex;
  m->nflexnode = nflexnode;
  m->nflexvert = nflexvert;
  m->nflexedge = nflexedge;
  m->nflexelem = nflexelem;
  m->nflexelemdata = nflexelemdata;
  m->nflexelemedge = nflexelemedge;
  m->nflexshelldata = nflexshelldata;
  m->nflexevpair = nflexevpair;
  m->nflextexcoord = nflextexcoord;
  m->nmesh = nmesh;
  m->nmeshvert = nmeshvert;
  m->nmeshnormal = nmeshnormal;
  m->nmeshtexcoord = nmeshtexcoord;
  m->nmeshface = nmeshface;
  m->nmeshgraph = nmeshgraph;
  m->nmeshpoly = nmeshpoly;
  m->nmeshpolyvert = nmeshpolyvert;
  m->nmeshpolymap = nmeshpolymap;
  m->nskin = nskin;
  m->nskinvert = nskinvert;
  m->nskintexvert = nskintexvert;
  m->nskinface = nskinface;
  m->nskinbone = nskinbone;
  m->nskinbonevert = nskinbonevert;
  m->nhfield = nhfield;
  m->nhfielddata = nhfielddata;
  m->ntex = ntex;
  m->ntexdata = ntexdata;
  m->nmat = nmat;
  m->npair = npair;
  m->nexclude = nexclude;
  m->neq = neq;
  m->ntendon = ntendon;
  m->nwrap = nwrap;
  m->nsensor = nsensor;
  m->nnumeric = nnumeric;
  m->nnumericdata = nnumericdata;
  m->ntext = ntext;
  m->ntextdata = ntextdata;
  m->ntuple = ntuple;
  m->ntupledata = ntupledata;
  m->nkey = nkey;
  m->nmocap = nmocap;
  m->nplugin = nplugin;
  m->npluginattr = npluginattr;
  m->nuser_body = nuser_body;
  m->nuser_jnt = nuser_jnt;
  m->nuser_geom = nuser_geom;
  m->nuser_site = nuser_site;
  m->nuser_cam = nuser_cam;
  m->nuser_tendon = nuser_tendon;
  m->nuser_actuator = nuser_actuator;
  m->nuser_sensor = nuser_sensor;
  m->nnames = nnames;
  long nnames_map = (long)nbody + njnt + ngeom + nsite + ncam + nlight + nflex + nmesh + nskin +
                    nhfield + ntex + nmat + npair + nexclude + neq + ntendon + nu + nsensor +
                    nnumeric + ntext + ntuple + nkey + nplugin;
  if (nnames_map >= INT_MAX / mjLOAD_MULTIPLE) {
    if (allocate) mju_free(m);
    mju_warning("Invalid model: size of nnames_map is larger than INT_MAX");
    return;
  }
  m->nnames_map = mjLOAD_MULTIPLE * nnames_map;
  m->npaths = npaths;

#define X(name)                                    \
  if ((m->name) < 0) {                             \
    if (allocate) mju_free(m);                     \
    mju_warning("Invalid model: negative " #name); \
    return;                                        \
  }
  MJMODEL_INTS;
#undef X

  // nbody should always be positive
  if (m->nbody == 0) {
    if (allocate) mju_free(m);
    mju_warning("Invalid model: nbody == 0");
    return;
  }

  // nmocap is going to get multiplied by 4, and shouldn't overflow
  if (m->nmocap >= MAX_ARRAY_SIZE) {
    if (allocate) mju_free(m);
    mju_warning("Invalid model: nmocap too large");
    return;
  }

  // compute buffer size
  m->nbuffer = 0;
#define X(type, name, nr, nc)                                                \
  if (!safeAddToBufferSize(&offset, &m->nbuffer, sizeof(type), m->nr, nc)) { \
    if (allocate) mju_free(m);                                               \
    mju_warning("Invalid model: " #name " too large.");                      \
    return;                                                                  \
  }

  MJMODEL_POINTERS
#undef X

  // allocate buffer
  m->buffer = mju_malloc(m->nbuffer);
  if (!m->buffer) {
    if (allocate) mju_free(m);
    mjERROR("could not allocate mjModel buffer");
  }

  // clear, set pointers in buffer
  memset(m->buffer, 0, m->nbuffer);
#ifdef MEMORY_SANITIZER
  // Tell msan to treat the entire buffer as uninitialized
  __msan_allocated_memory(m->buffer, m->nbuffer);
#endif
  mj_setPtrModel(m);

  // set default options
  mj_defaultOption(&m->opt);
  mj_defaultVisual(&m->vis);
  mj_defaultStatistic(&m->stat);

  // copy pointer if allocated here
  if (allocate) {
    *dest = m;
  }
}


// de-allocate mjModel
void mj_deleteModel(mjModel* m) {
    if (m) {
        freeModelBuffers(m);
        mju_free(m);
    }
}

// de-allocate mjData
void mj_deleteData(mjData* d) {
    if (d) {
        freeDataBuffers(d);
        mju_free(d);
    }
}


// load binary MJB model
mjModel* mj_loadModelBuffer(const void* buffer, int buffer_sz, CTcTrace m_Trace) {
    int header[NHEADER] = { 0 };
    int expected_header[NHEADER] = { ID, sizeof(mjtNum), getnint(), getnsize(), getnptr() };
    int ints[256];
    size_t sizes[8];
    int ptrbuf = 0;
    mjModel* m = 0;

    if (buffer_sz < NHEADER * sizeof(int)) {
        mju_warning(m_Trace, "Model file has an incomplete header");
        return NULL;
    }

    bufread(header, NHEADER * sizeof(int), buffer_sz, buffer, &ptrbuf);

    // check header
    for (int i = 0; i < NHEADER; i++) {
        if (header[i] != expected_header[i]) {
            switch (i) {
            case 0:
                mju_warning(m_Trace, "Model missing header ID");
                return NULL;

            case 1:
                mju_warning(m_Trace, "Model and executable have different floating point precision");
                return NULL;

            case 2:
                mju_warning(m_Trace, "Model and executable have different number of ints in mjModel");
                return NULL;

            case 3:
                mju_warning(m_Trace, "Model and executable have different number of size_t members in mjModel");
                return NULL;

            default:
                mju_warning(m_Trace, "Model and executable have different number of pointers in mjModel");
                return NULL;
            }
        }
    }
    // read mjModel structure: info only
    if (ptrbuf + sizeof(int) * getnint() + sizeof(size_t) * getnsize() > buffer_sz) {
        mju_warning(m_Trace, "Truncated model file - ran out of data while reading sizes");
        return NULL;
    }

    bufread(ints, sizeof(int) * getnint(), buffer_sz, buffer, &ptrbuf);
    bufread(sizes, sizeof(size_t) * getnsize(), buffer_sz, buffer, &ptrbuf);

    // allocate new mjModel, check sizes
    mj_makeModel(&m,
        ints[0],  ints[1],  ints[2],  ints[3],  ints[4],  ints[5],  ints[6],
        ints[7],  ints[8],  ints[9],  ints[10], ints[11], ints[12], ints[13],
        ints[14], ints[15], ints[16], ints[17], ints[18], ints[19], ints[20],
        ints[21], ints[22], ints[23], ints[24], ints[25], ints[26], ints[27],
        ints[28], ints[29], ints[30], ints[31], ints[32], ints[33], ints[34],
        ints[35], ints[36], ints[37], ints[38], ints[39], ints[40], ints[41],
        ints[42], ints[43], ints[44], ints[45], ints[46], ints[47], ints[48],
        ints[49], ints[50], ints[51], ints[52], ints[53], ints[54], ints[55],
        ints[56], ints[57], ints[58], ints[59], ints[60], ints[61], ints[62],
        ints[63], ints[64], ints[65], ints[66], ints[67], ints[68]);
    if (!m || m->nbuffer != sizes[getnsize() - 1]) {
        mju_warning(m_Trace, "Corrupted model, wrong size parameters");
        mj_deleteModel(m);
        return NULL;
    }

    // set integer fields
    {
        int int_idx = 0;
        int size_idx = 0;
#define X(name) if(std::is_same<decltype(m->name), size_t>::value) m->name = sizes[size_idx++]; else m->name=ints[int_idx++];
        //#define X(name) \
        //        m->name = _Generic(m->name, size_t: sizes[size_idx++], default: ints[int_idx++]);
        MJMODEL_INTS
#undef X
    }

    // read options and buffer
    if (ptrbuf + sizeof(mjOption) + sizeof(mjVisual) + sizeof(mjStatistic) > buffer_sz) {
        mju_warning(m_Trace, "Truncated model file - ran out of data while reading structs");
        return NULL;
    }
    bufread((void*)&m->opt, sizeof(mjOption), buffer_sz, buffer, &ptrbuf);
    bufread((void*)&m->vis, sizeof(mjVisual), buffer_sz, buffer, &ptrbuf);
    bufread((void*)&m->stat, sizeof(mjStatistic), buffer_sz, buffer, &ptrbuf);
    {
        MJMODEL_POINTERS_PREAMBLE(m)
#define X(type, name, nr, nc)                                           \
      if (ptrbuf + sizeof(type) * (m->nr) * (nc) > buffer_sz) {             \
        mju_warning(m_Trace,                                                         \
            "Truncated model file - ran out of data while reading " #name); \
        mj_deleteModel(m);                                                  \
        return NULL;                                                        \
      }                                                                     \
      bufread(m->name, sizeof(type)*(m->nr)*(nc), buffer_sz, buffer, &ptrbuf);

            MJMODEL_POINTERS
#undef X
    }

    // make sure buffer is the correct size
    if (ptrbuf != buffer_sz) {
        mju_warning(m_Trace, "Model file is too large");
        mj_deleteModel(m);
        return NULL;
    }

    //TODOOO
    // const char* validationError = mj_validateReferences(m);
    const char* validationError = NULL;
    if (validationError) {
        mju_warning(m_Trace, "%s", validationError);
        mj_deleteModel(m);
        return NULL;
    }

    return m;
}


// TC: Commeneted out as multi-threading is not supported
static void maybe_lock_alloc_mutex(mjData* d) {
//   if (d->threadpool != 0) {
//     mju_threadPoolLockAllocMutex((mjThreadPool*)d->threadpool);
//   }
}


// TC: Commeneted out as multi-threading is not supported
static void maybe_unlock_alloc_mutex(mjData* d) {
//   if (d->threadpool != 0) {
//     mju_threadPoolUnlockAllocMutex((mjThreadPool*)d->threadpool);
//   }
}


// stack allocation / freeing
static inline mjStackInfo get_stack_info_from_data(const mjData* d) {
    mjStackInfo stack_info;
    stack_info.bottom = (uintptr_t)d->arena + (uintptr_t)d->narena;
    stack_info.top = stack_info.bottom - d->pstack;
    stack_info.limit = (uintptr_t)d->arena + (uintptr_t)d->parena;
    stack_info.stack_base = d->pbase;

    return stack_info;
}

// allocate memory from the mjData arena
void* mj_arenaAllocByte(mjData* d, size_t bytes, size_t alignment) {
  maybe_lock_alloc_mutex(d);
  size_t misalignment = fastmod(d->parena, alignment);
  size_t padding = misalignment ? alignment - misalignment : 0;

  // check size
  size_t bytes_available = d->narena - d->pstack;
  if (mjUNLIKELY(d->parena + padding + bytes > bytes_available)) {
    maybe_unlock_alloc_mutex(d);
    return NULL;
  }

  size_t stack_usage = d->pstack;

  // under ASAN, get stack usage from red zone
#ifdef ADDRESS_SANITIZER
  mjStackInfo stack_info;
  mjStackInfo* stack_info_ptr;
  if (!d->threadpool) {
    stack_info = get_stack_info_from_data(d);
    stack_info_ptr = &stack_info;
  } else {
    size_t thread_id = mju_threadPoolCurrentWorkerId((mjThreadPool*)d->threadpool);
    stack_info_ptr = mju_getStackInfoForThread(d, thread_id);
  }
  stack_usage = stack_usage_redzone(stack_info_ptr);
#endif

  // allocate, update max, return pointer to buffer
  void* result = (char*)d->arena + d->parena + padding;
  d->parena += padding + bytes;
  d->maxuse_arena = mjMAX(d->maxuse_arena, stack_usage + d->parena);

#ifdef ADDRESS_SANITIZER
  ASAN_UNPOISON_MEMORY_REGION(result, bytes);
#endif

#ifdef MEMORY_SANITIZER
  __msan_allocated_memory(result, bytes);
#endif

  maybe_unlock_alloc_mutex(d);
  return result;
}



// internal: allocate size bytes on the provided stack shard
// declared inline so that modular arithmetic with specific alignments can be optimized out
// TC: No implementation of thread pools
static inline void* stackallocinternal(mjData* d, mjStackInfo* stack_info, size_t size,
    size_t alignment, const char* caller, int line) {
    // return NULL if empty
    if (mjUNLIKELY(!size)) {
        return NULL;
    }

    // start of the memory to be allocated to the buffer
    uintptr_t start_ptr = stack_info->top - (size + mjREDZONE);

    // align the pointer
    start_ptr -= fastmod(start_ptr, alignment);

    // new top of the stack
    uintptr_t new_top_ptr = start_ptr - mjREDZONE;

    // exclude red zone from stack usage statistics
    size_t current_alloc_usage = stack_info->top - new_top_ptr - 2 * mjREDZONE;
    size_t usage = current_alloc_usage + (stack_info->bottom - stack_info->top);

    // check size
    size_t stack_available_bytes = stack_info->top - stack_info->limit;
    size_t stack_required_bytes = stack_info->top - new_top_ptr;
    if (mjUNLIKELY(stack_required_bytes > stack_available_bytes)) {
        char info[1024];
        if (caller) {
            snprintf(info, sizeof(info), " at %s, line %d", caller, line);
        } else {
            info[0] = '\0';
        }
        mju_error("mj_stackAlloc: out of memory, stack overflow%s\n"
                "  max = %zu, available = %zu, requested = %zu\n"
                "  nefc = %d, ncon = %d",
                info,
                stack_info->bottom - stack_info->limit, stack_available_bytes, stack_required_bytes,
                d->nefc, d->ncon);
    }

#ifdef ADDRESS_SANITIZER
    usage = current_alloc_usage + stack_usage_redzone(stack_info);

    // store new stack usage in the red zone
    size_t misalign = new_top_ptr % _Alignof(size_t);
    size_t* usage_ptr =
        (size_t*)(new_top_ptr + (misalign ? _Alignof(size_t) - misalign : 0));
    ASAN_UNPOISON_MEMORY_REGION(usage_ptr, sizeof(size_t));
    *usage_ptr = usage;
    ASAN_POISON_MEMORY_REGION(usage_ptr, sizeof(size_t));

    // unpoison the actual usable allocation
    ASAN_UNPOISON_MEMORY_REGION((void*)start_ptr, size);
#endif

    // update max usage statistics
    stack_info->top = new_top_ptr;
    //if (!d->threadpool) {
        d->maxuse_stack = mjMAX(d->maxuse_stack, usage);
        d->maxuse_arena = mjMAX(d->maxuse_arena, usage + d->parena);
    //}
    //else {
    //    size_t thread_id = mju_threadPoolCurrentWorkerId((mjThreadPool*)d->threadpool);
    //    d->maxuse_threadstack[thread_id] = mjMAX(d->maxuse_threadstack[thread_id], usage);
    //}

    return (void*)start_ptr;
}



// internal: allocate size bytes in mjData
// declared inline so that modular arithmetic with specific alignments can be optimized out
// TC: No implementation of thread pools
static inline void* stackalloc(mjData* d, size_t size, size_t alignment,
    const char* caller, int line) {
    //if (!d->threadpool) {
        mjStackInfo stack_info = get_stack_info_from_data(d);

        void* result = stackallocinternal(d, &stack_info, size, alignment, caller, line);

        d->pstack = stack_info.bottom - stack_info.top;

        return result;
    //}

    //size_t thread_id = mju_threadPoolCurrentWorkerId((mjThreadPool*)d->threadpool);
    //mjStackInfo* stack_info = mju_getStackInfoForThread(d, thread_id);
    //return stackallocinternal(d, stack_info, size, alignment);
}

// allocate bytes on the stack, with caller information
void* mj_stackAllocInfo(mjData* d, size_t bytes, size_t alignment,
    const char* caller, int line) {
return stackalloc(d, bytes, alignment, caller, line);
}


// TC: changed _Alignof(mjtNum) to 1
// allocate mjtNums on the stack
mjtNum* mj_stackAllocNum(mjData* d, size_t size) {
    if (mjUNLIKELY(size >= SIZE_MAX / sizeof(mjtNum))) {
        mjERROR("requested size is too large (more than 2^64 bytes).");
    }
    //return (mjtNum*)stackalloc(d, size * sizeof(mjtNum), _Alignof(mjtNum));
    return (mjtNum*) stackalloc(d, size * sizeof(mjtNum), 1, NULL, 0);

}


// TC: changed _Alignof(mjtNum) to 1
// allocate ints on the stack
int* mj_stackAllocInt(mjData* d, size_t size) {
    if (mjUNLIKELY(size >= SIZE_MAX / sizeof(int))) {
        mjERROR("requested size is too large (more than 2^64 bytes).");
    }
    //return (int*) stackalloc(d, size * sizeof(int), _Alignof(int));
    return (int*) stackalloc(d, size * sizeof(int), 1, NULL, 0);

}


// mjStackInfo mark stack frame, inline so ASAN errors point to correct code unit
// TC: Changed _Alignof(mjtNum) and _Alignof(mjStackFrame) to 1
#ifdef ADDRESS_SANITIZER
__attribute__((always_inline))
#endif
static inline void markstackinternal(mjData* d, mjStackInfo* stack_info) {
    size_t top_old = stack_info->top;
    mjStackFrame* s =
        (mjStackFrame*) stackallocinternal(d, stack_info, sizeof(mjStackFrame), 1, NULL, 0);
    s->pbase = stack_info->stack_base;
    s->pstack = top_old;
#ifdef ADDRESS_SANITIZER
    // store the program counter to the caller so that we can compare against mj_freeStack later
    s->pc = __sanitizer_return_address();
#endif
    stack_info->stack_base = (uintptr_t)s;
}



// mjData mark stack frame
// TC: No implementation of thread pools
#ifndef ADDRESS_SANITIZER
void mj_markStack(mjData* d)
#else
void mj__markStack(mjData* d)
#endif
{
    //if (!d->threadpool) {
        mjStackInfo stack_info = get_stack_info_from_data(d);
        markstackinternal(d, &stack_info);
        d->pstack = stack_info.bottom - stack_info.top;
        d->pbase = stack_info.stack_base;
        return;
    //}

    //size_t thread_id = mju_threadPoolCurrentWorkerId((mjThreadPool*)d->threadpool);
    //mjStackInfo* stack_info = mju_getStackInfoForThread(d, thread_id);
    //markstackinternal(d, stack_info);
}



#ifdef ADDRESS_SANITIZER
__attribute__((always_inline))
#endif
static inline void freestackinternal(mjStackInfo* stack_info) {
    if (mjUNLIKELY(!stack_info->stack_base)) {
        return;
    }

    mjStackFrame* s = (mjStackFrame*)stack_info->stack_base;
#ifdef ADDRESS_SANITIZER
    // raise an error if caller function name doesn't match the most recent caller of mj_markStack
    if (!mj__comparePcFuncName(s->pc, __sanitizer_return_address())) {
        mjERROR("mj_markStack %s has no corresponding mj_freeStack (detected %s)",
            mj__getPcDebugInfo(s->pc),
            mj__getPcDebugInfo(__sanitizer_return_address()));
    }
#endif

    // restore pbase and pstack
    stack_info->stack_base = s->pbase;
    stack_info->top = s->pstack;

    // if running under asan, poison the newly freed memory region
#ifdef ADDRESS_SANITIZER
    ASAN_POISON_MEMORY_REGION((char*)stack_info->limit, stack_info->top - stack_info->limit);
#endif
}



// mjData free stack frame
// TC: No implementation of thread pools
#ifndef ADDRESS_SANITIZER
void mj_freeStack(mjData* d)
#else
void mj__freeStack(mjData* d)
#endif
{
    //if (!d->threadpool) {
        mjStackInfo stack_info = get_stack_info_from_data(d);
        freestackinternal(&stack_info);
        d->pstack = stack_info.bottom - stack_info.top;
        d->pbase = stack_info.stack_base;
        return;
    //}

    //size_t thread_id = mju_threadPoolCurrentWorkerId((mjThreadPool*)d->threadpool);
    //mjStackInfo* stack_info = mju_getStackInfoForThread(d, thread_id);
    //freestackinternal(stack_info);
}
