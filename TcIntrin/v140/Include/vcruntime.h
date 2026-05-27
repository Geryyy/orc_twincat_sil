#pragma once

#ifndef TC_SDK_ONLY
	#define _Out_
	#define _In_
	#define _Check_return_ 
#else
	#include "sal.h"
#endif

#ifdef __cplusplus
#define _CRT_BEGIN_C_HEADER extern "C" {
#define _CRT_END_C_HEADER }
#else
#define _CRT_BEGIN_C_HEADER 
#define _CRT_END_C_HEADER 
#endif

#define _VCRT_ALIGN(X) __declspec(align(X))
