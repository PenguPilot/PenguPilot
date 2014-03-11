/* Produced by CVXGEN, 2014-08-28 01:29:22 -0700.  */
/* CVXGEN is Copyright (C) 2006-2014 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2014 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */

#include "solver.h"

void multbymA(float *lhs, float *rhs) {
(void)lhs;
(void)rhs;
}

void multbymAT(float *lhs, float *rhs) {
  (void)rhs;
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
}

void multbymG(float *lhs, float *rhs) {
  lhs[0] = -rhs[0]*(params.A[0])-rhs[1]*(params.A[8])-rhs[2]*(params.A[16])-rhs[3]*(params.A[24]);
  lhs[1] = -rhs[0]*(params.A[1])-rhs[1]*(params.A[9])-rhs[2]*(params.A[17])-rhs[3]*(params.A[25]);
  lhs[2] = -rhs[0]*(params.A[2])-rhs[1]*(params.A[10])-rhs[2]*(params.A[18])-rhs[3]*(params.A[26]);
  lhs[3] = -rhs[0]*(params.A[3])-rhs[1]*(params.A[11])-rhs[2]*(params.A[19])-rhs[3]*(params.A[27]);
  lhs[4] = -rhs[0]*(params.A[4])-rhs[1]*(params.A[12])-rhs[2]*(params.A[20])-rhs[3]*(params.A[28]);
  lhs[5] = -rhs[0]*(params.A[5])-rhs[1]*(params.A[13])-rhs[2]*(params.A[21])-rhs[3]*(params.A[29]);
  lhs[6] = -rhs[0]*(params.A[6])-rhs[1]*(params.A[14])-rhs[2]*(params.A[22])-rhs[3]*(params.A[30]);
  lhs[7] = -rhs[0]*(params.A[7])-rhs[1]*(params.A[15])-rhs[2]*(params.A[23])-rhs[3]*(params.A[31]);
}

void multbymGT(float *lhs, float *rhs) {
  lhs[0] = -rhs[0]*(params.A[0])-rhs[1]*(params.A[1])-rhs[2]*(params.A[2])-rhs[3]*(params.A[3])-rhs[4]*(params.A[4])-rhs[5]*(params.A[5])-rhs[6]*(params.A[6])-rhs[7]*(params.A[7]);
  lhs[1] = -rhs[0]*(params.A[8])-rhs[1]*(params.A[9])-rhs[2]*(params.A[10])-rhs[3]*(params.A[11])-rhs[4]*(params.A[12])-rhs[5]*(params.A[13])-rhs[6]*(params.A[14])-rhs[7]*(params.A[15]);
  lhs[2] = -rhs[0]*(params.A[16])-rhs[1]*(params.A[17])-rhs[2]*(params.A[18])-rhs[3]*(params.A[19])-rhs[4]*(params.A[20])-rhs[5]*(params.A[21])-rhs[6]*(params.A[22])-rhs[7]*(params.A[23]);
  lhs[3] = -rhs[0]*(params.A[24])-rhs[1]*(params.A[25])-rhs[2]*(params.A[26])-rhs[3]*(params.A[27])-rhs[4]*(params.A[28])-rhs[5]*(params.A[29])-rhs[6]*(params.A[30])-rhs[7]*(params.A[31]);
}

void multbyP(float *lhs, float *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.G[0])+rhs[1]*(2*params.G[4])+rhs[2]*(2*params.G[8])+rhs[3]*(2*params.G[12]);
  lhs[1] = rhs[0]*(2*params.G[1])+rhs[1]*(2*params.G[5])+rhs[2]*(2*params.G[9])+rhs[3]*(2*params.G[13]);
  lhs[2] = rhs[0]*(2*params.G[2])+rhs[1]*(2*params.G[6])+rhs[2]*(2*params.G[10])+rhs[3]*(2*params.G[14]);
  lhs[3] = rhs[0]*(2*params.G[3])+rhs[1]*(2*params.G[7])+rhs[2]*(2*params.G[11])+rhs[3]*(2*params.G[15]);
}

void fillq(void) {
  work.q[0] = -2*(params.G[0]*params.xd[0]+params.G[4]*params.xd[1]+params.G[8]*params.xd[2]+params.G[12]*params.xd[3]);
  work.q[1] = -2*(params.G[1]*params.xd[0]+params.G[5]*params.xd[1]+params.G[9]*params.xd[2]+params.G[13]*params.xd[3]);
  work.q[2] = -2*(params.G[2]*params.xd[0]+params.G[6]*params.xd[1]+params.G[10]*params.xd[2]+params.G[14]*params.xd[3]);
  work.q[3] = -2*(params.G[3]*params.xd[0]+params.G[7]*params.xd[1]+params.G[11]*params.xd[2]+params.G[15]*params.xd[3]);
}

void fillh(void) {
  work.h[0] = params.b[0];
  work.h[1] = params.b[1];
  work.h[2] = params.b[2];
  work.h[3] = params.b[3];
  work.h[4] = params.b[4];
  work.h[5] = params.b[5];
  work.h[6] = params.b[6];
  work.h[7] = params.b[7];
}

void fillb(void) {
}

void pre_ops(void) {
  work.quad_342101520384[0] = (params.xd[0]*(params.G[0]*params.xd[0]+params.G[4]*params.xd[1]+params.G[8]*params.xd[2]+params.G[12]*params.xd[3])+params.xd[1]*(params.G[1]*params.xd[0]+params.G[5]*params.xd[1]+params.G[9]*params.xd[2]+params.G[13]*params.xd[3])+params.xd[2]*(params.G[2]*params.xd[0]+params.G[6]*params.xd[1]+params.G[10]*params.xd[2]+params.G[14]*params.xd[3])+params.xd[3]*(params.G[3]*params.xd[0]+params.G[7]*params.xd[1]+params.G[11]*params.xd[2]+params.G[15]*params.xd[3]));

}
