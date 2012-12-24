/*___________________________________________________
 |  _____                       _____ _ _       _    |
 | |  __ \                     |  __ (_) |     | |   |
 | | |__) |__ _ __   __ _ _   _| |__) || | ___ | |_  |
 | |  ___/ _ \ '_ \ / _` | | | |  ___/ | |/ _ \| __| |
 | | |  |  __/ | | | (_| | |_| | |   | | | (_) | |_  |
 | |_|   \___|_| |_|\__, |\__,_|_|   |_|_|\___/ \__| |
 |                   __/ |                           |
 |  GNU/Linux based |___/  Multi-Rotor UAV Autopilot |
 |___________________________________________________|
  
 File Purpose

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */

/* Produced by CVXGEN, 2012-08-28 01:29:22 -0700.  */
/* CVXGEN is Copyright (C) 2006-2011 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2011 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: ldl.c. */
/* Description: Basic test harness for solver.c. */

#include "solver.h"

/* Be sure to place ldl_solve first, so storage schemes are defined by it. */
void ldl_solve(float *target, float *var) {
  int i;
  /* Find var = (L*diag(work.d)*L') \ target, then unpermute. */
  /* Answer goes into var. */

  /* Forward substitution. */
  /* Include permutation as we retrieve from target. Use v so we can unpermute */
  /* later. */
  work.v[0] = target[4];
  work.v[1] = target[5];
  work.v[2] = target[6];
  work.v[3] = target[7];
  work.v[4] = target[8];
  work.v[5] = target[9];
  work.v[6] = target[10];
  work.v[7] = target[11];
  work.v[8] = target[12]-work.L[0]*work.v[0];
  work.v[9] = target[13]-work.L[1]*work.v[1];
  work.v[10] = target[14]-work.L[2]*work.v[2];
  work.v[11] = target[15]-work.L[3]*work.v[3];
  work.v[12] = target[16]-work.L[4]*work.v[4];
  work.v[13] = target[17]-work.L[5]*work.v[5];
  work.v[14] = target[18]-work.L[6]*work.v[6];
  work.v[15] = target[0]-work.L[7]*work.v[8]-work.L[8]*work.v[9]-work.L[9]*work.v[10]-work.L[10]*work.v[11]-work.L[11]*work.v[12]-work.L[12]*work.v[13]-work.L[13]*work.v[14];
  work.v[16] = target[1]-work.L[14]*work.v[8]-work.L[15]*work.v[9]-work.L[16]*work.v[10]-work.L[17]*work.v[11]-work.L[18]*work.v[12]-work.L[19]*work.v[13]-work.L[20]*work.v[14]-work.L[21]*work.v[15];
  work.v[17] = target[2]-work.L[22]*work.v[8]-work.L[23]*work.v[9]-work.L[24]*work.v[10]-work.L[25]*work.v[11]-work.L[26]*work.v[12]-work.L[27]*work.v[13]-work.L[28]*work.v[14]-work.L[29]*work.v[15]-work.L[30]*work.v[16];
  work.v[18] = target[3]-work.L[31]*work.v[8]-work.L[32]*work.v[9]-work.L[33]*work.v[10]-work.L[34]*work.v[11]-work.L[35]*work.v[12]-work.L[36]*work.v[13]-work.L[37]*work.v[14]-work.L[38]*work.v[15]-work.L[39]*work.v[16]-work.L[40]*work.v[17];
  work.v[19] = target[19]-work.L[41]*work.v[7]-work.L[42]*work.v[15]-work.L[43]*work.v[16]-work.L[44]*work.v[17]-work.L[45]*work.v[18];
  /* Diagonal scaling. Assume correctness of work.d_inv. */
  for (i = 0; i < 20; i++)
    work.v[i] *= work.d_inv[i];
  /* Back substitution */
  work.v[18] -= work.L[45]*work.v[19];
  work.v[17] -= work.L[40]*work.v[18]+work.L[44]*work.v[19];
  work.v[16] -= work.L[30]*work.v[17]+work.L[39]*work.v[18]+work.L[43]*work.v[19];
  work.v[15] -= work.L[21]*work.v[16]+work.L[29]*work.v[17]+work.L[38]*work.v[18]+work.L[42]*work.v[19];
  work.v[14] -= work.L[13]*work.v[15]+work.L[20]*work.v[16]+work.L[28]*work.v[17]+work.L[37]*work.v[18];
  work.v[13] -= work.L[12]*work.v[15]+work.L[19]*work.v[16]+work.L[27]*work.v[17]+work.L[36]*work.v[18];
  work.v[12] -= work.L[11]*work.v[15]+work.L[18]*work.v[16]+work.L[26]*work.v[17]+work.L[35]*work.v[18];
  work.v[11] -= work.L[10]*work.v[15]+work.L[17]*work.v[16]+work.L[25]*work.v[17]+work.L[34]*work.v[18];
  work.v[10] -= work.L[9]*work.v[15]+work.L[16]*work.v[16]+work.L[24]*work.v[17]+work.L[33]*work.v[18];
  work.v[9] -= work.L[8]*work.v[15]+work.L[15]*work.v[16]+work.L[23]*work.v[17]+work.L[32]*work.v[18];
  work.v[8] -= work.L[7]*work.v[15]+work.L[14]*work.v[16]+work.L[22]*work.v[17]+work.L[31]*work.v[18];
  work.v[7] -= work.L[41]*work.v[19];
  work.v[6] -= work.L[6]*work.v[14];
  work.v[5] -= work.L[5]*work.v[13];
  work.v[4] -= work.L[4]*work.v[12];
  work.v[3] -= work.L[3]*work.v[11];
  work.v[2] -= work.L[2]*work.v[10];
  work.v[1] -= work.L[1]*work.v[9];
  work.v[0] -= work.L[0]*work.v[8];
  /* Unpermute the result, from v to var. */
  var[0] = work.v[15];
  var[1] = work.v[16];
  var[2] = work.v[17];
  var[3] = work.v[18];
  var[4] = work.v[0];
  var[5] = work.v[1];
  var[6] = work.v[2];
  var[7] = work.v[3];
  var[8] = work.v[4];
  var[9] = work.v[5];
  var[10] = work.v[6];
  var[11] = work.v[7];
  var[12] = work.v[8];
  var[13] = work.v[9];
  var[14] = work.v[10];
  var[15] = work.v[11];
  var[16] = work.v[12];
  var[17] = work.v[13];
  var[18] = work.v[14];
  var[19] = work.v[19];
#ifndef ZERO_LIBRARY_MODE
  if (settings.debug) {
    printf("Squared norm for solution is %.8g.\n", check_residual(target, var));
  }
#endif
}

void ldl_factor(void) {
  work.d[0] = work.KKT[0];
  if (work.d[0] < 0)
    work.d[0] = settings.kkt_reg;
  else
    work.d[0] += settings.kkt_reg;
  work.d_inv[0] = 1/work.d[0];

  work.L[0] = work.KKT[1]*work.d_inv[0];

  work.v[1] = work.KKT[2];
  work.d[1] = work.v[1];

  if (work.d[1] < 0)
    work.d[1] = settings.kkt_reg;
  else
    work.d[1] += settings.kkt_reg;
  work.d_inv[1] = 1/work.d[1];

  work.L[1] = (work.KKT[3])*work.d_inv[1];
  work.v[2] = work.KKT[4];
  work.d[2] = work.v[2];

  if (work.d[2] < 0)
    work.d[2] = settings.kkt_reg;
  else
    work.d[2] += settings.kkt_reg;
  work.d_inv[2] = 1/work.d[2];

  work.L[2] = (work.KKT[5])*work.d_inv[2];
  work.v[3] = work.KKT[6];
  work.d[3] = work.v[3];

  if (work.d[3] < 0)
    work.d[3] = settings.kkt_reg;
  else
    work.d[3] += settings.kkt_reg;
  work.d_inv[3] = 1/work.d[3];

  work.L[3] = (work.KKT[7])*work.d_inv[3];
  work.v[4] = work.KKT[8];
  work.d[4] = work.v[4];

  if (work.d[4] < 0)
    work.d[4] = settings.kkt_reg;
  else
    work.d[4] += settings.kkt_reg;
  work.d_inv[4] = 1/work.d[4];

  work.L[4] = (work.KKT[9])*work.d_inv[4];
  work.v[5] = work.KKT[10];
  work.d[5] = work.v[5];

  if (work.d[5] < 0)
    work.d[5] = settings.kkt_reg;
  else
    work.d[5] += settings.kkt_reg;
  work.d_inv[5] = 1/work.d[5];

  work.L[5] = (work.KKT[11])*work.d_inv[5];
  work.v[6] = work.KKT[12];
  work.d[6] = work.v[6];

  if (work.d[6] < 0)
    work.d[6] = settings.kkt_reg;
  else
    work.d[6] += settings.kkt_reg;
  work.d_inv[6] = 1/work.d[6];

  work.L[6] = (work.KKT[13])*work.d_inv[6];
  work.v[7] = work.KKT[14];
  work.d[7] = work.v[7];

  if (work.d[7] < 0)
    work.d[7] = settings.kkt_reg;
  else
    work.d[7] += settings.kkt_reg;
  work.d_inv[7] = 1/work.d[7];

  work.L[41] = (work.KKT[15])*work.d_inv[7];
  work.v[0] = work.L[0]*work.d[0];
  work.v[8] = work.KKT[16]-work.L[0]*work.v[0];
  work.d[8] = work.v[8];

  if (work.d[8] > 0)
    work.d[8] = -settings.kkt_reg;
  else
    work.d[8] -= settings.kkt_reg;

  work.d_inv[8] = 1/work.d[8];

  work.L[7] = (work.KKT[17])*work.d_inv[8];
  work.L[14] = (work.KKT[18])*work.d_inv[8];
  work.L[22] = (work.KKT[19])*work.d_inv[8];
  work.L[31] = (work.KKT[20])*work.d_inv[8];
  work.v[1] = work.L[1]*work.d[1];
  work.v[9] = work.KKT[21]-work.L[1]*work.v[1];
  work.d[9] = work.v[9];

  if (work.d[9] > 0)
    work.d[9] = -settings.kkt_reg;
  else
    work.d[9] -= settings.kkt_reg;

  work.d_inv[9] = 1/work.d[9];

  work.L[8] = (work.KKT[22])*work.d_inv[9];
  work.L[15] = (work.KKT[23])*work.d_inv[9];
  work.L[23] = (work.KKT[24])*work.d_inv[9];
  work.L[32] = (work.KKT[25])*work.d_inv[9];
  work.v[2] = work.L[2]*work.d[2];
  work.v[10] = work.KKT[26]-work.L[2]*work.v[2];
  work.d[10] = work.v[10];

  if (work.d[10] > 0)
    work.d[10] = -settings.kkt_reg;
  else
    work.d[10] -= settings.kkt_reg;

  work.d_inv[10] = 1/work.d[10];

  work.L[9] = (work.KKT[27])*work.d_inv[10];
  work.L[16] = (work.KKT[28])*work.d_inv[10];
  work.L[24] = (work.KKT[29])*work.d_inv[10];
  work.L[33] = (work.KKT[30])*work.d_inv[10];
  work.v[3] = work.L[3]*work.d[3];
  work.v[11] = work.KKT[31]-work.L[3]*work.v[3];
  work.d[11] = work.v[11];

  if (work.d[11] > 0)
    work.d[11] = -settings.kkt_reg;
  else
    work.d[11] -= settings.kkt_reg;

  work.d_inv[11] = 1/work.d[11];

  work.L[10] = (work.KKT[32])*work.d_inv[11];
  work.L[17] = (work.KKT[33])*work.d_inv[11];
  work.L[25] = (work.KKT[34])*work.d_inv[11];
  work.L[34] = (work.KKT[35])*work.d_inv[11];
  work.v[4] = work.L[4]*work.d[4];
  work.v[12] = work.KKT[36]-work.L[4]*work.v[4];
  work.d[12] = work.v[12];

  if (work.d[12] > 0)
    work.d[12] = -settings.kkt_reg;
  else
    work.d[12] -= settings.kkt_reg;

  work.d_inv[12] = 1/work.d[12];

  work.L[11] = (work.KKT[37])*work.d_inv[12];
  work.L[18] = (work.KKT[38])*work.d_inv[12];
  work.L[26] = (work.KKT[39])*work.d_inv[12];
  work.L[35] = (work.KKT[40])*work.d_inv[12];
  work.v[5] = work.L[5]*work.d[5];
  work.v[13] = work.KKT[41]-work.L[5]*work.v[5];
  work.d[13] = work.v[13];

  if (work.d[13] > 0)
    work.d[13] = -settings.kkt_reg;
  else
    work.d[13] -= settings.kkt_reg;

  work.d_inv[13] = 1/work.d[13];

  work.L[12] = (work.KKT[42])*work.d_inv[13];
  work.L[19] = (work.KKT[43])*work.d_inv[13];
  work.L[27] = (work.KKT[44])*work.d_inv[13];
  work.L[36] = (work.KKT[45])*work.d_inv[13];
  work.v[6] = work.L[6]*work.d[6];
  work.v[14] = work.KKT[46]-work.L[6]*work.v[6];
  work.d[14] = work.v[14];

  if (work.d[14] > 0)
    work.d[14] = -settings.kkt_reg;
  else
    work.d[14] -= settings.kkt_reg;

  work.d_inv[14] = 1/work.d[14];

  work.L[13] = (work.KKT[47])*work.d_inv[14];
  work.L[20] = (work.KKT[48])*work.d_inv[14];
  work.L[28] = (work.KKT[49])*work.d_inv[14];
  work.L[37] = (work.KKT[50])*work.d_inv[14];
  work.v[8] = work.L[7]*work.d[8];
  work.v[9] = work.L[8]*work.d[9];
  work.v[10] = work.L[9]*work.d[10];
  work.v[11] = work.L[10]*work.d[11];
  work.v[12] = work.L[11]*work.d[12];
  work.v[13] = work.L[12]*work.d[13];
  work.v[14] = work.L[13]*work.d[14];
  work.v[15] = work.KKT[51]-work.L[7]*work.v[8]-work.L[8]*work.v[9]-work.L[9]*work.v[10]-work.L[10]*work.v[11]-work.L[11]*work.v[12]-work.L[12]*work.v[13]-work.L[13]*work.v[14];
  work.d[15] = work.v[15];

  if (work.d[15] < 0)
    work.d[15] = settings.kkt_reg;
  else
    work.d[15] += settings.kkt_reg;
  work.d_inv[15] = 1/work.d[15];

  work.L[21] = (work.KKT[52]-work.L[14]*work.v[8]-work.L[15]*work.v[9]-work.L[16]*work.v[10]-work.L[17]*work.v[11]-work.L[18]*work.v[12]-work.L[19]*work.v[13]-work.L[20]*work.v[14])*work.d_inv[15];
  work.L[29] = (work.KKT[53]-work.L[22]*work.v[8]-work.L[23]*work.v[9]-work.L[24]*work.v[10]-work.L[25]*work.v[11]-work.L[26]*work.v[12]-work.L[27]*work.v[13]-work.L[28]*work.v[14])*work.d_inv[15];
  work.L[38] = (work.KKT[54]-work.L[31]*work.v[8]-work.L[32]*work.v[9]-work.L[33]*work.v[10]-work.L[34]*work.v[11]-work.L[35]*work.v[12]-work.L[36]*work.v[13]-work.L[37]*work.v[14])*work.d_inv[15];
  work.L[42] = (work.KKT[55])*work.d_inv[15];
  work.v[8] = work.L[14]*work.d[8];
  work.v[9] = work.L[15]*work.d[9];
  work.v[10] = work.L[16]*work.d[10];
  work.v[11] = work.L[17]*work.d[11];
  work.v[12] = work.L[18]*work.d[12];
  work.v[13] = work.L[19]*work.d[13];
  work.v[14] = work.L[20]*work.d[14];
  work.v[15] = work.L[21]*work.d[15];
  work.v[16] = work.KKT[56]-work.L[14]*work.v[8]-work.L[15]*work.v[9]-work.L[16]*work.v[10]-work.L[17]*work.v[11]-work.L[18]*work.v[12]-work.L[19]*work.v[13]-work.L[20]*work.v[14]-work.L[21]*work.v[15];
  work.d[16] = work.v[16];

  if (work.d[16] < 0)
    work.d[16] = settings.kkt_reg;
  else
    work.d[16] += settings.kkt_reg;
  work.d_inv[16] = 1/work.d[16];

  work.L[30] = (work.KKT[57]-work.L[22]*work.v[8]-work.L[23]*work.v[9]-work.L[24]*work.v[10]-work.L[25]*work.v[11]-work.L[26]*work.v[12]-work.L[27]*work.v[13]-work.L[28]*work.v[14]-work.L[29]*work.v[15])*work.d_inv[16];
  work.L[39] = (work.KKT[58]-work.L[31]*work.v[8]-work.L[32]*work.v[9]-work.L[33]*work.v[10]-work.L[34]*work.v[11]-work.L[35]*work.v[12]-work.L[36]*work.v[13]-work.L[37]*work.v[14]-work.L[38]*work.v[15])*work.d_inv[16];
  work.L[43] = (work.KKT[59]-work.L[42]*work.v[15])*work.d_inv[16];
  work.v[8] = work.L[22]*work.d[8];
  work.v[9] = work.L[23]*work.d[9];
  work.v[10] = work.L[24]*work.d[10];
  work.v[11] = work.L[25]*work.d[11];
  work.v[12] = work.L[26]*work.d[12];
  work.v[13] = work.L[27]*work.d[13];
  work.v[14] = work.L[28]*work.d[14];
  work.v[15] = work.L[29]*work.d[15];
  work.v[16] = work.L[30]*work.d[16];
  work.v[17] = work.KKT[60]-work.L[22]*work.v[8]-work.L[23]*work.v[9]-work.L[24]*work.v[10]-work.L[25]*work.v[11]-work.L[26]*work.v[12]-work.L[27]*work.v[13]-work.L[28]*work.v[14]-work.L[29]*work.v[15]-work.L[30]*work.v[16];
  work.d[17] = work.v[17];

  if (work.d[17] < 0)
    work.d[17] = settings.kkt_reg;
  else
    work.d[17] += settings.kkt_reg;
  work.d_inv[17] = 1/work.d[17];

  work.L[40] = (work.KKT[61]-work.L[31]*work.v[8]-work.L[32]*work.v[9]-work.L[33]*work.v[10]-work.L[34]*work.v[11]-work.L[35]*work.v[12]-work.L[36]*work.v[13]-work.L[37]*work.v[14]-work.L[38]*work.v[15]-work.L[39]*work.v[16])*work.d_inv[17];
  work.L[44] = (work.KKT[62]-work.L[42]*work.v[15]-work.L[43]*work.v[16])*work.d_inv[17];
  work.v[8] = work.L[31]*work.d[8];
  work.v[9] = work.L[32]*work.d[9];
  work.v[10] = work.L[33]*work.d[10];
  work.v[11] = work.L[34]*work.d[11];
  work.v[12] = work.L[35]*work.d[12];
  work.v[13] = work.L[36]*work.d[13];
  work.v[14] = work.L[37]*work.d[14];
  work.v[15] = work.L[38]*work.d[15];
  work.v[16] = work.L[39]*work.d[16];
  work.v[17] = work.L[40]*work.d[17];
  work.v[18] = work.KKT[63]-work.L[31]*work.v[8]-work.L[32]*work.v[9]-work.L[33]*work.v[10]-work.L[34]*work.v[11]-work.L[35]*work.v[12]-work.L[36]*work.v[13]-work.L[37]*work.v[14]-work.L[38]*work.v[15]-work.L[39]*work.v[16]-work.L[40]*work.v[17];
  work.d[18] = work.v[18];

  if (work.d[18] < 0)
    work.d[18] = settings.kkt_reg;
  else
    work.d[18] += settings.kkt_reg;
  work.d_inv[18] = 1/work.d[18];

  work.L[45] = (work.KKT[64]-work.L[42]*work.v[15]-work.L[43]*work.v[16]-work.L[44]*work.v[17])*work.d_inv[18];
  work.v[7] = work.L[41]*work.d[7];
  work.v[15] = work.L[42]*work.d[15];
  work.v[16] = work.L[43]*work.d[16];
  work.v[17] = work.L[44]*work.d[17];
  work.v[18] = work.L[45]*work.d[18];
  work.v[19] = work.KKT[65]-work.L[41]*work.v[7]-work.L[42]*work.v[15]-work.L[43]*work.v[16]-work.L[44]*work.v[17]-work.L[45]*work.v[18];
  work.d[19] = work.v[19];

  if (work.d[19] > 0)
    work.d[19] = -settings.kkt_reg;
  else
    work.d[19] -= settings.kkt_reg;

  work.d_inv[19] = 1/work.d[19];

#ifndef ZERO_LIBRARY_MODE
  if (settings.debug) {
    printf("Squared Frobenius for factorization is %.8g.\n", check_factorization());
  }
#endif
}

float check_factorization(void) {
  /* Returns the squared Frobenius norm of A - L*D*L'. */
  float temp, residual;

  /* Only check the lower triangle. */
  residual = 0;
  temp = work.KKT[51]-1*work.d[15]*1-work.L[7]*work.d[8]*work.L[7]-work.L[8]*work.d[9]*work.L[8]-work.L[9]*work.d[10]*work.L[9]-work.L[10]*work.d[11]*work.L[10]-work.L[11]*work.d[12]*work.L[11]-work.L[12]*work.d[13]*work.L[12]-work.L[13]*work.d[14]*work.L[13];
  residual += temp*temp;

  temp = work.KKT[52]-work.L[21]*work.d[15]*1-work.L[14]*work.d[8]*work.L[7]-work.L[15]*work.d[9]*work.L[8]-work.L[16]*work.d[10]*work.L[9]-work.L[17]*work.d[11]*work.L[10]-work.L[18]*work.d[12]*work.L[11]-work.L[19]*work.d[13]*work.L[12]-work.L[20]*work.d[14]*work.L[13];
  residual += temp*temp;

  temp = work.KKT[53]-work.L[29]*work.d[15]*1-work.L[22]*work.d[8]*work.L[7]-work.L[23]*work.d[9]*work.L[8]-work.L[24]*work.d[10]*work.L[9]-work.L[25]*work.d[11]*work.L[10]-work.L[26]*work.d[12]*work.L[11]-work.L[27]*work.d[13]*work.L[12]-work.L[28]*work.d[14]*work.L[13];
  residual += temp*temp;

  temp = work.KKT[54]-work.L[38]*work.d[15]*1-work.L[31]*work.d[8]*work.L[7]-work.L[32]*work.d[9]*work.L[8]-work.L[33]*work.d[10]*work.L[9]-work.L[34]*work.d[11]*work.L[10]-work.L[35]*work.d[12]*work.L[11]-work.L[36]*work.d[13]*work.L[12]-work.L[37]*work.d[14]*work.L[13];
  residual += temp*temp;

  temp = work.KKT[56]-work.L[21]*work.d[15]*work.L[21]-1*work.d[16]*1-work.L[14]*work.d[8]*work.L[14]-work.L[15]*work.d[9]*work.L[15]-work.L[16]*work.d[10]*work.L[16]-work.L[17]*work.d[11]*work.L[17]-work.L[18]*work.d[12]*work.L[18]-work.L[19]*work.d[13]*work.L[19]-work.L[20]*work.d[14]*work.L[20];
  residual += temp*temp;

  temp = work.KKT[57]-work.L[29]*work.d[15]*work.L[21]-work.L[30]*work.d[16]*1-work.L[22]*work.d[8]*work.L[14]-work.L[23]*work.d[9]*work.L[15]-work.L[24]*work.d[10]*work.L[16]-work.L[25]*work.d[11]*work.L[17]-work.L[26]*work.d[12]*work.L[18]-work.L[27]*work.d[13]*work.L[19]-work.L[28]*work.d[14]*work.L[20];
  residual += temp*temp;

  temp = work.KKT[58]-work.L[38]*work.d[15]*work.L[21]-work.L[39]*work.d[16]*1-work.L[31]*work.d[8]*work.L[14]-work.L[32]*work.d[9]*work.L[15]-work.L[33]*work.d[10]*work.L[16]-work.L[34]*work.d[11]*work.L[17]-work.L[35]*work.d[12]*work.L[18]-work.L[36]*work.d[13]*work.L[19]-work.L[37]*work.d[14]*work.L[20];
  residual += temp*temp;

  temp = work.KKT[60]-work.L[29]*work.d[15]*work.L[29]-work.L[30]*work.d[16]*work.L[30]-1*work.d[17]*1-work.L[22]*work.d[8]*work.L[22]-work.L[23]*work.d[9]*work.L[23]-work.L[24]*work.d[10]*work.L[24]-work.L[25]*work.d[11]*work.L[25]-work.L[26]*work.d[12]*work.L[26]-work.L[27]*work.d[13]*work.L[27]-work.L[28]*work.d[14]*work.L[28];
  residual += temp*temp;

  temp = work.KKT[61]-work.L[38]*work.d[15]*work.L[29]-work.L[39]*work.d[16]*work.L[30]-work.L[40]*work.d[17]*1-work.L[31]*work.d[8]*work.L[22]-work.L[32]*work.d[9]*work.L[23]-work.L[33]*work.d[10]*work.L[24]-work.L[34]*work.d[11]*work.L[25]-work.L[35]*work.d[12]*work.L[26]-work.L[36]*work.d[13]*work.L[27]-work.L[37]*work.d[14]*work.L[28];
  residual += temp*temp;

  temp = work.KKT[63]-work.L[38]*work.d[15]*work.L[38]-work.L[39]*work.d[16]*work.L[39]-work.L[40]*work.d[17]*work.L[40]-1*work.d[18]*1-work.L[31]*work.d[8]*work.L[31]-work.L[32]*work.d[9]*work.L[32]-work.L[33]*work.d[10]*work.L[33]-work.L[34]*work.d[11]*work.L[34]-work.L[35]*work.d[12]*work.L[35]-work.L[36]*work.d[13]*work.L[36]-work.L[37]*work.d[14]*work.L[37];
  residual += temp*temp;

  temp = work.KKT[0]-1*work.d[0]*1;
  residual += temp*temp;

  temp = work.KKT[2]-1*work.d[1]*1;
  residual += temp*temp;

  temp = work.KKT[4]-1*work.d[2]*1;
  residual += temp*temp;

  temp = work.KKT[6]-1*work.d[3]*1;
  residual += temp*temp;

  temp = work.KKT[8]-1*work.d[4]*1;
  residual += temp*temp;

  temp = work.KKT[10]-1*work.d[5]*1;
  residual += temp*temp;

  temp = work.KKT[12]-1*work.d[6]*1;
  residual += temp*temp;

  temp = work.KKT[14]-1*work.d[7]*1;
  residual += temp*temp;

  temp = work.KKT[1]-work.L[0]*work.d[0]*1;
  residual += temp*temp;

  temp = work.KKT[3]-work.L[1]*work.d[1]*1;
  residual += temp*temp;

  temp = work.KKT[5]-work.L[2]*work.d[2]*1;
  residual += temp*temp;

  temp = work.KKT[7]-work.L[3]*work.d[3]*1;
  residual += temp*temp;

  temp = work.KKT[9]-work.L[4]*work.d[4]*1;
  residual += temp*temp;

  temp = work.KKT[11]-work.L[5]*work.d[5]*1;
  residual += temp*temp;

  temp = work.KKT[13]-work.L[6]*work.d[6]*1;
  residual += temp*temp;

  temp = work.KKT[15]-work.L[41]*work.d[7]*1;
  residual += temp*temp;

  temp = work.KKT[16]-work.L[0]*work.d[0]*work.L[0]-1*work.d[8]*1;
  residual += temp*temp;

  temp = work.KKT[21]-work.L[1]*work.d[1]*work.L[1]-1*work.d[9]*1;
  residual += temp*temp;

  temp = work.KKT[26]-work.L[2]*work.d[2]*work.L[2]-1*work.d[10]*1;
  residual += temp*temp;

  temp = work.KKT[31]-work.L[3]*work.d[3]*work.L[3]-1*work.d[11]*1;
  residual += temp*temp;

  temp = work.KKT[36]-work.L[4]*work.d[4]*work.L[4]-1*work.d[12]*1;
  residual += temp*temp;

  temp = work.KKT[41]-work.L[5]*work.d[5]*work.L[5]-1*work.d[13]*1;
  residual += temp*temp;

  temp = work.KKT[46]-work.L[6]*work.d[6]*work.L[6]-1*work.d[14]*1;
  residual += temp*temp;

  temp = work.KKT[65]-work.L[41]*work.d[7]*work.L[41]-1*work.d[19]*1-work.L[42]*work.d[15]*work.L[42]-work.L[43]*work.d[16]*work.L[43]-work.L[44]*work.d[17]*work.L[44]-work.L[45]*work.d[18]*work.L[45];
  residual += temp*temp;

  temp = work.KKT[17]-1*work.d[8]*work.L[7];
  residual += temp*temp;

  temp = work.KKT[18]-1*work.d[8]*work.L[14];
  residual += temp*temp;

  temp = work.KKT[19]-1*work.d[8]*work.L[22];
  residual += temp*temp;

  temp = work.KKT[20]-1*work.d[8]*work.L[31];
  residual += temp*temp;

  temp = work.KKT[22]-1*work.d[9]*work.L[8];
  residual += temp*temp;

  temp = work.KKT[23]-1*work.d[9]*work.L[15];
  residual += temp*temp;

  temp = work.KKT[24]-1*work.d[9]*work.L[23];
  residual += temp*temp;

  temp = work.KKT[25]-1*work.d[9]*work.L[32];
  residual += temp*temp;

  temp = work.KKT[27]-1*work.d[10]*work.L[9];
  residual += temp*temp;

  temp = work.KKT[28]-1*work.d[10]*work.L[16];
  residual += temp*temp;

  temp = work.KKT[29]-1*work.d[10]*work.L[24];
  residual += temp*temp;

  temp = work.KKT[30]-1*work.d[10]*work.L[33];
  residual += temp*temp;

  temp = work.KKT[32]-1*work.d[11]*work.L[10];
  residual += temp*temp;

  temp = work.KKT[33]-1*work.d[11]*work.L[17];
  residual += temp*temp;

  temp = work.KKT[34]-1*work.d[11]*work.L[25];
  residual += temp*temp;

  temp = work.KKT[35]-1*work.d[11]*work.L[34];
  residual += temp*temp;

  temp = work.KKT[37]-1*work.d[12]*work.L[11];
  residual += temp*temp;

  temp = work.KKT[38]-1*work.d[12]*work.L[18];
  residual += temp*temp;

  temp = work.KKT[39]-1*work.d[12]*work.L[26];
  residual += temp*temp;

  temp = work.KKT[40]-1*work.d[12]*work.L[35];
  residual += temp*temp;

  temp = work.KKT[42]-1*work.d[13]*work.L[12];
  residual += temp*temp;

  temp = work.KKT[43]-1*work.d[13]*work.L[19];
  residual += temp*temp;

  temp = work.KKT[44]-1*work.d[13]*work.L[27];
  residual += temp*temp;

  temp = work.KKT[45]-1*work.d[13]*work.L[36];
  residual += temp*temp;

  temp = work.KKT[47]-1*work.d[14]*work.L[13];
  residual += temp*temp;

  temp = work.KKT[48]-1*work.d[14]*work.L[20];
  residual += temp*temp;

  temp = work.KKT[49]-1*work.d[14]*work.L[28];
  residual += temp*temp;

  temp = work.KKT[50]-1*work.d[14]*work.L[37];
  residual += temp*temp;

  temp = work.KKT[55]-work.L[42]*work.d[15]*1;
  residual += temp*temp;

  temp = work.KKT[59]-work.L[42]*work.d[15]*work.L[21]-work.L[43]*work.d[16]*1;
  residual += temp*temp;

  temp = work.KKT[62]-work.L[42]*work.d[15]*work.L[29]-work.L[43]*work.d[16]*work.L[30]-work.L[44]*work.d[17]*1;
  residual += temp*temp;

  temp = work.KKT[64]-work.L[42]*work.d[15]*work.L[38]-work.L[43]*work.d[16]*work.L[39]-work.L[44]*work.d[17]*work.L[40]-work.L[45]*work.d[18]*1;
  residual += temp*temp;

  return residual;
}

void matrix_multiply(float *result, float *source) {
  /* Finds result = A*source. */

  result[0] = work.KKT[51]*source[0]+work.KKT[52]*source[1]+work.KKT[53]*source[2]+work.KKT[54]*source[3]+work.KKT[17]*source[12]+work.KKT[22]*source[13]+work.KKT[27]*source[14]+work.KKT[32]*source[15]+work.KKT[37]*source[16]+work.KKT[42]*source[17]+work.KKT[47]*source[18]+work.KKT[55]*source[19];
  result[1] = work.KKT[52]*source[0]+work.KKT[56]*source[1]+work.KKT[57]*source[2]+work.KKT[58]*source[3]+work.KKT[18]*source[12]+work.KKT[23]*source[13]+work.KKT[28]*source[14]+work.KKT[33]*source[15]+work.KKT[38]*source[16]+work.KKT[43]*source[17]+work.KKT[48]*source[18]+work.KKT[59]*source[19];
  result[2] = work.KKT[53]*source[0]+work.KKT[57]*source[1]+work.KKT[60]*source[2]+work.KKT[61]*source[3]+work.KKT[19]*source[12]+work.KKT[24]*source[13]+work.KKT[29]*source[14]+work.KKT[34]*source[15]+work.KKT[39]*source[16]+work.KKT[44]*source[17]+work.KKT[49]*source[18]+work.KKT[62]*source[19];
  result[3] = work.KKT[54]*source[0]+work.KKT[58]*source[1]+work.KKT[61]*source[2]+work.KKT[63]*source[3]+work.KKT[20]*source[12]+work.KKT[25]*source[13]+work.KKT[30]*source[14]+work.KKT[35]*source[15]+work.KKT[40]*source[16]+work.KKT[45]*source[17]+work.KKT[50]*source[18]+work.KKT[64]*source[19];
  result[4] = work.KKT[0]*source[4]+work.KKT[1]*source[12];
  result[5] = work.KKT[2]*source[5]+work.KKT[3]*source[13];
  result[6] = work.KKT[4]*source[6]+work.KKT[5]*source[14];
  result[7] = work.KKT[6]*source[7]+work.KKT[7]*source[15];
  result[8] = work.KKT[8]*source[8]+work.KKT[9]*source[16];
  result[9] = work.KKT[10]*source[9]+work.KKT[11]*source[17];
  result[10] = work.KKT[12]*source[10]+work.KKT[13]*source[18];
  result[11] = work.KKT[14]*source[11]+work.KKT[15]*source[19];
  result[12] = work.KKT[1]*source[4]+work.KKT[16]*source[12]+work.KKT[17]*source[0]+work.KKT[18]*source[1]+work.KKT[19]*source[2]+work.KKT[20]*source[3];
  result[13] = work.KKT[3]*source[5]+work.KKT[21]*source[13]+work.KKT[22]*source[0]+work.KKT[23]*source[1]+work.KKT[24]*source[2]+work.KKT[25]*source[3];
  result[14] = work.KKT[5]*source[6]+work.KKT[26]*source[14]+work.KKT[27]*source[0]+work.KKT[28]*source[1]+work.KKT[29]*source[2]+work.KKT[30]*source[3];
  result[15] = work.KKT[7]*source[7]+work.KKT[31]*source[15]+work.KKT[32]*source[0]+work.KKT[33]*source[1]+work.KKT[34]*source[2]+work.KKT[35]*source[3];
  result[16] = work.KKT[9]*source[8]+work.KKT[36]*source[16]+work.KKT[37]*source[0]+work.KKT[38]*source[1]+work.KKT[39]*source[2]+work.KKT[40]*source[3];
  result[17] = work.KKT[11]*source[9]+work.KKT[41]*source[17]+work.KKT[42]*source[0]+work.KKT[43]*source[1]+work.KKT[44]*source[2]+work.KKT[45]*source[3];
  result[18] = work.KKT[13]*source[10]+work.KKT[46]*source[18]+work.KKT[47]*source[0]+work.KKT[48]*source[1]+work.KKT[49]*source[2]+work.KKT[50]*source[3];
  result[19] = work.KKT[15]*source[11]+work.KKT[65]*source[19]+work.KKT[55]*source[0]+work.KKT[59]*source[1]+work.KKT[62]*source[2]+work.KKT[64]*source[3];
}

float check_residual(float *target, float *multiplicand) {
  /* Returns the squared 2-norm of lhs - A*rhs. */
  /* Reuses v to find the residual. */
  int i;
  float residual;

  residual = 0;
  matrix_multiply(work.v, multiplicand);
  for (i = 0; i < 4; i++) {
    residual += (target[i] - work.v[i])*(target[i] - work.v[i]);
  }
  return residual;
}

void fill_KKT(void) {
  work.KKT[51] = 2*params.G[0];
  work.KKT[52] = 2*params.G[4];
  work.KKT[53] = 2*params.G[8];
  work.KKT[54] = 2*params.G[12];
  work.KKT[56] = 2*params.G[5];
  work.KKT[57] = 2*params.G[9];
  work.KKT[58] = 2*params.G[13];
  work.KKT[60] = 2*params.G[10];
  work.KKT[61] = 2*params.G[14];
  work.KKT[63] = 2*params.G[15];
  work.KKT[0] = work.s_inv_z[0];
  work.KKT[2] = work.s_inv_z[1];
  work.KKT[4] = work.s_inv_z[2];
  work.KKT[6] = work.s_inv_z[3];
  work.KKT[8] = work.s_inv_z[4];
  work.KKT[10] = work.s_inv_z[5];
  work.KKT[12] = work.s_inv_z[6];
  work.KKT[14] = work.s_inv_z[7];
  work.KKT[1] = 1;
  work.KKT[3] = 1;
  work.KKT[5] = 1;
  work.KKT[7] = 1;
  work.KKT[9] = 1;
  work.KKT[11] = 1;
  work.KKT[13] = 1;
  work.KKT[15] = 1;
  work.KKT[16] = work.block_33[0];
  work.KKT[21] = work.block_33[0];
  work.KKT[26] = work.block_33[0];
  work.KKT[31] = work.block_33[0];
  work.KKT[36] = work.block_33[0];
  work.KKT[41] = work.block_33[0];
  work.KKT[46] = work.block_33[0];
  work.KKT[65] = work.block_33[0];
  work.KKT[17] = params.A[0];
  work.KKT[18] = params.A[8];
  work.KKT[19] = params.A[16];
  work.KKT[20] = params.A[24];
  work.KKT[22] = params.A[1];
  work.KKT[23] = params.A[9];
  work.KKT[24] = params.A[17];
  work.KKT[25] = params.A[25];
  work.KKT[27] = params.A[2];
  work.KKT[28] = params.A[10];
  work.KKT[29] = params.A[18];
  work.KKT[30] = params.A[26];
  work.KKT[32] = params.A[3];
  work.KKT[33] = params.A[11];
  work.KKT[34] = params.A[19];
  work.KKT[35] = params.A[27];
  work.KKT[37] = params.A[4];
  work.KKT[38] = params.A[12];
  work.KKT[39] = params.A[20];
  work.KKT[40] = params.A[28];
  work.KKT[42] = params.A[5];
  work.KKT[43] = params.A[13];
  work.KKT[44] = params.A[21];
  work.KKT[45] = params.A[29];
  work.KKT[47] = params.A[6];
  work.KKT[48] = params.A[14];
  work.KKT[49] = params.A[22];
  work.KKT[50] = params.A[30];
  work.KKT[55] = params.A[7];
  work.KKT[59] = params.A[15];
  work.KKT[62] = params.A[23];
  work.KKT[64] = params.A[31];
}
