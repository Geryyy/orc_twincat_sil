#pragma once

#include "mujoco_tc/engine_util_misc_tc.h"

#include <string>
#include "mujoco_tc/mujoco_tc.h"
#include "mujoco_tc/mjmodel.h"
#include "mujoco_tc/engine_util_blas_tc.h"
#include "mujoco_tc/mjtnum.h"
#include "mujoco_tc/engine_util_spatial_tc.h"


// return 1 if point is outside box given by pos, mat, size * inflate
// return -1 if point is inside box given by pos, mat, size / inflate
// return 0 if point is between the inflated and deflated boxes
int mju_outsideBox(const mjtNum point[3], const mjtNum pos[3], const mjtNum mat[9],
                   const mjtNum size[3], mjtNum inflate) {
  // check inflation coefficient
  if (inflate < 1) {
    mjERROR("inflation coefficient must be >= 1")
  }

  // vector from pos to point, projected to box frame
  mjtNum vec[3] = {point[0]-pos[0], point[1]-pos[1], point[2]-pos[2]};
  mju_mulMatTVec3(vec, mat, vec);

  // big: inflated box
  mjtNum big[3] = {size[0], size[1], size[2]};
  if (inflate > 1) {
    mju_scl3(big, big, inflate);
  }

  // check if outside big box
  if (vec[0] > big[0] || vec[0] < -big[0] ||
      vec[1] > big[1] || vec[1] < -big[1] ||
      vec[2] > big[2] || vec[2] < -big[2]) {
    return 1;
  }

  // quick return if no inflation
  if (inflate == 1) {
    return -1;
  }

  // check if inside small (deflated) box
  mjtNum small[3] = {size[0]/inflate, size[1]/inflate, size[2]/inflate};
  if (vec[0] < small[0] && vec[0] > -small[0] &&
      vec[1] < small[1] && vec[1] > -small[1] &&
      vec[2] < small[2] && vec[2] > -small[2]) {
    return -1;
  }

  // within margin between small and big box
  return 0;
}



// min function, avoid re-evaluation
mjtNum mju_min(mjtNum a, mjtNum b) {
  if (a <= b) {
    return a;
  } else {
    return b;
  }
}



// max function, avoid re-evaluation
mjtNum mju_max(mjtNum a, mjtNum b) {
  if (a >= b) {
    return a;
  } else {
    return b;
  }
}



// clip x to the range [min, max]
mjtNum mju_clip(mjtNum x, mjtNum min, mjtNum max) {
  if (x < min) {
    return min;
  } else if (x > max) {
    return max;
  } else {
    return x;
  }
}



// sign function
mjtNum mju_sign(mjtNum x) {
  if (x < 0) {
    return -1;
  } else if (x > 0) {
    return 1;
  } else {
    return 0;
  }
}



// return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise
int mju_isBad(mjtNum x) {
  return (x != x || x > mjMAXVAL || x < -mjMAXVAL);
}



// return 1 if all elements are 0
int mju_isZero(mjtNum* vec, int n) {
  for (int i=0; i < n; i++) {
    if (vec[i] != 0) {
      return 0;
    }
  }

  return 1;
}



// set integer vector to 0
void mju_zeroInt(int* res, int n) {
  memset(res, 0, n*sizeof(int));
}


void mju_zeroSizeT(size_t* res, size_t n) {
  memset(res, 0, n*sizeof(size_t));
}


// copy int vector vec into res
void mju_copyInt(int* res, const int* vec, int n) {
  memcpy(res, vec, n*sizeof(int));
}



//// standard normal random number generator (optional second number)
//mjtNum mju_standardNormal(mjtNum* num2) {
//  const mjtNum scale = 2.0/((mjtNum)RAND_MAX);
//  mjtNum x1, x2, w;
//
//  do {
//    x1 = scale * (mjtNum)rand() - 1.0;
//    x2 = scale * (mjtNum)rand() - 1.0;
//    w = x1 * x1 + x2 * x2;
//  } while (w >= 1.0 || w == 0);
//
//  w = mju_sqrt((-2.0 * mju_log(w)) / w);
//  if (num2) {
//    *num2 = x2 * w;
//  }
//
//  return (x1 * w);
//}



// convert from float to mjtNum
void mju_f2n(mjtNum* res, const float* vec, int n) {
  for (int i=0; i < n; i++) {
    res[i] = (mjtNum) vec[i];
  }
}



// convert from mjtNum to float
void mju_n2f(float* res, const mjtNum* vec, int n) {
  for (int i=0; i < n; i++) {
    res[i] = (float) vec[i];
  }
}


// convert from double to mjtNum
void mju_d2n(mjtNum* res, const double* vec, int n) {
  for (int i=0; i < n; i++) {
    res[i] = (mjtNum) vec[i];
  }
}



// convert from mjtNum to double
void mju_n2d(double* res, const mjtNum* vec, int n) {
  for (int i=0; i < n; i++) {
    res[i] = (double) vec[i];
  }
}

// gather
// TC: Removed restrict
void mju_gather(mjtNum* res, const mjtNum* vec, const int* ind, int n) {
  for (int i=0; i < n; i++) {
    res[i] = vec[ind[i]];
  }
}



// scatter
// TC: Removed restrict
void mju_scatter(mjtNum* res, const mjtNum* vec, const int* ind, int n) {
  for (int i=0; i < n; i++) {
    res[ind[i]] = vec[i];
  }
}



// insertion sort, increasing order
void mju_insertionSort(mjtNum* list, int n) {
  for (int i=1; i < n; i++) {
    mjtNum x = list[i];
    int j = i-1;
    while (j >= 0 && list[j] > x) {
      list[j+1] = list[j];
      j--;
    }
    list[j+1] = x;
  }
}



// integer insertion sort, increasing order
void mju_insertionSortInt(int* list, int n) {
  for (int i=1; i < n; i++) {
    int x = list[i];
    int j = i-1;
    while (j >= 0 && list[j] > x) {
      list[j+1] = list[j];
      j--;
    }
    list[j+1] = x;
  }
}



// Halton sequence
mjtNum mju_Halton(int index, int base) {
  int n0 = index;
  mjtNum b = (mjtNum)base;
  mjtNum f = 1/b, hn = 0;

  while (n0 > 0) {
    int n1 = n0/base;
    int r = n0 - n1*base;
    hn += f*r;
    f /= b;
    n0 = n1;
  }

  return hn;
}



// Call strncpy, then set dst[n-1] = 0.
char* mju_strncpy(char *dst, const char *src, int n) {
  if (dst && src && n > 0) {
    strncpy(dst, src, n);
    dst[n-1] = 0;
  }

  return dst;
}



// sigmoid function over 0<=x<=1 using quintic polynomial
mjtNum mju_sigmoid(mjtNum x) {
  // fast return
  if (x <= 0) {
    return 0;
  }
  if (x >= 1) {
    return 1;
  }

  // sigmoid: f(x) = 6*x^5 - 15*x^4 + 10*x^3
  // solution of f(0) = f'(0) = f''(0) = 0, f(1) = 1, f'(1) = f''(1) = 0
  return x*x*x * (3*x * (2*x - 5) + 10);
}
