#pragma once

#include "sal.h"
#include "crtdefs.h"

#ifdef __cplusplus
#define _CRT_BEGIN_C_HEADER \
	extern "C" { \
	__pragma(pack(push, _CRT_PACKING))

#define _CRT_END_C_HEADER \
	}\
	__pragma(pack(pop))

#else
#define _CRT_BEGIN_C_HEADER __pragma(pack(push, _CRT_PACKING))
#define _CRT_END_C_HEADER __pragma(pack(pop))
#endif

#define _VCRT_ALIGN(X) __declspec(align(X))
