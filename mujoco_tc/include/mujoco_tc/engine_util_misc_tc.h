#pragma once

#include <string>
#include <stddef.h>
#include <stdint.h>

//using namespace std;
#include "TcInterfaces.h"
#include "TcRtInterfaces.h"
#include "mujoco_tc/mjtnum.h"

//------------------------------ miscellaneous ----------------------------------------------------

// return 1 if point is outside box given by pos, mat, size * inflate
// return -1 if point is inside box given by pos, mat, size / inflate
// return 0 if point is between the inflated and deflated boxes
int mju_outsideBox(const mjtNum point[3], const mjtNum pos[3], const mjtNum mat[9],
                         const mjtNum size[3], mjtNum inflate);

//print matrix to the TC logger
inline void mju_printMat(const mjtNum* mat, int nr, int nc, CTcTrace m_Trace) {
	using namespace std;
	for (int r = 0; r < nr; r++) {
		char row_string[1024] = "";
		for (int c = 0; c < nc; c++) {
			char buffer[100];
			snprintf(buffer, sizeof(buffer), "%.8f ", mat[nc * r + c]);
			strcat(row_string, buffer);
		}
		m_Trace.Log(tlAlways, row_string);
	}
}

//print matrix to the TC logger
inline void mju_printMat_row_major(const mjtNum* mat, int nr, int nc, CTcTrace m_Trace) {
	using namespace std;
	for (int r = 0; r < nr; r++) {
		char row_string[1024] = "";
		for (int c = 0; c < nc; c++) {
			char buffer[100];
			snprintf(buffer, sizeof(buffer), "%.8f ", mat[nr * c + r]);
			strcat(row_string, buffer);
		}
		m_Trace.Log(tlAlways, row_string);
	}
}


// print vector to the TC logger
inline void mju_printVec(const mjtNum* vec, int n, CTcTrace m_Trace) {
	using namespace std;
	char result[1024] = "";
	for (int i = 0; i < n; i++) {
		char buffer[100];
		snprintf(buffer, sizeof(buffer), "%.8f ", vec[i]);
		strcat(result, buffer);
	}
	m_Trace.Log(tlAlways, result);
}

// min function, single evaluation of a and b
mjtNum mju_min(mjtNum a, mjtNum b);

// max function, single evaluation of a and b
mjtNum mju_max(mjtNum a, mjtNum b);

// clip x to the range [min, max]
mjtNum mju_clip(mjtNum x, mjtNum min, mjtNum max);

// sign function
mjtNum mju_sign(mjtNum x);

// round to nearest integer
int mju_round(mjtNum x);

// return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise
int mju_isBad(mjtNum x);

// return 1 if all elements are 0
int mju_isZero(mjtNum* vec, int n);

// set integer vector to 0
void mju_zeroInt(int* res, int n);

// set size_t vector to 0
void mju_zeroSizeT(size_t* res, size_t n);

// copy int vector vec into res
void mju_copyInt(int* res, const int* vec, int n);

//// standard normal random number generator (optional second number)
//mjtNum mju_standardNormal(mjtNum* num2);

// convert from float to mjtNum
void mju_f2n(mjtNum* res, const float* vec, int n);

// convert from mjtNum to float
void mju_n2f(float* res, const mjtNum* vec, int n);

// convert from double to mjtNum
void mju_d2n(mjtNum* res, const double* vec, int n);

// convert from mjtNum to double
void mju_n2d(double* res, const mjtNum* vec, int n);

// gather
void mju_gather(mjtNum* res, const mjtNum* vec, const int* ind, int n);

// scatter
void mju_scatter(mjtNum* res, const mjtNum* vec, const int* ind, int n);

// insertion sort, increasing order
void mju_insertionSort(mjtNum* list, int n);

// integer insertion sort, increasing order
void mju_insertionSortInt(int* list, int n);

// Halton sequence
mjtNum mju_Halton(int index, int base);

// call strncpy, then set dst[n-1] = 0
char* mju_strncpy(char *dst, const char *src, int n);

// sigmoid function over 0<=x<=1 using quintic polynomial
mjtNum mju_sigmoid(mjtNum x);