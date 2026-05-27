#pragma once

#include "mujoco_tc/mujoco_tc.h"

// derivative of mju_mulInertVec w.r.t vel
static void mjd_mulInertVec_vel(mjtNum D[36], const mjtNum i[10]) {
	mju_zero(D, 36);

	// res[0] = i[0]*v[0] + i[3]*v[1] + i[4]*v[2] - i[8]*v[4] + i[7]*v[5]
	D[0 + 0] = i[0];
	D[0 + 1] = i[3];
	D[0 + 2] = i[4];
	D[0 + 4] = -i[8];
	D[0 + 5] = i[7];

	// res[1] = i[3]*v[0] + i[1]*v[1] + i[5]*v[2] + i[8]*v[3] - i[6]*v[5]
	D[6 + 0] = i[3];
	D[6 + 1] = i[1];
	D[6 + 2] = i[5];
	D[6 + 3] = i[8];
	D[6 + 5] = -i[6];

	// res[2] = i[4]*v[0] + i[5]*v[1] + i[2]*v[2] - i[7]*v[3] + i[6]*v[4]
	D[12 + 0] = i[4];
	D[12 + 1] = i[5];
	D[12 + 2] = i[2];
	D[12 + 3] = -i[7];
	D[12 + 4] = i[6];

	// res[3] = i[8]*v[1] - i[7]*v[2] + i[9]*v[3]
	D[18 + 1] = i[8];
	D[18 + 2] = -i[7];
	D[18 + 3] = i[9];

	// res[4] = i[6]*v[2] - i[8]*v[0] + i[9]*v[4]
	D[24 + 2] = i[6];
	D[24 + 0] = -i[8];
	D[24 + 4] = i[9];

	// res[5] = i[7]*v[0] - i[6]*v[1] + i[9]*v[5]
	D[30 + 0] = i[7];
	D[30 + 1] = -i[6];
	D[30 + 5] = i[9];
}
