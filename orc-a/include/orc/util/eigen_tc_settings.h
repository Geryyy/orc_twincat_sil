/*
    Project: ORC - Open Robot Control Library
    Author: anonymous
    Date: Tue Jun 28 10:44:59 2022 +0200
    License: See accompanying LICENSE file
    */

#pragma once

#ifdef TC_VER
#include "intrin.h"
#include "malloc.h"
// #define TCMATH_BLOCK_STANDARDLIB (0)
#define EIGEN_HAS_CXX11_ATOMIC (0)
#define EIGEN_HAS_CXX11_MATH (0)

#define EIGEN_MAX_CPP_VER (11)

// #define EIGEN_HAS_CXX14 (0)
// #define EIGEN_NO_MALLOC (1)
#define EIGEN_STACK_ALLOCATION_LIMIT (0)
// #define atomic
// #define TCMATH_REDIRECT_STANDARDLIB
// #define _STL_DISABLE_CLANG_WARNINGS
// #include <sstream>

#define EIGEN_NO_CPUID
// #define _WIN32_WCE
#define EIGEN_NO_IO
// #define EIGEN_DONT_VECTORIZE
// #define EIGEN_DONT_PARALLELIZE
// #define EIGEN_MAX_CPP_VER 17
////Eigen compatilite definition
#define eigen_assert(x)
float rintf(const float& x);
int rint(const float& x);
long double fabsl(const long double& x);
float fmaxf(const float& f1, const float& f2);
float fminf(const float& f1, const float& f2);

void* _alloca(size_t size);
#endif
