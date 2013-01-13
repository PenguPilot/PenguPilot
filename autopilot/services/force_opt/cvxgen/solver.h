/* Produced by CVXGEN, 2012-08-28 01:29:22 -0700.  */
/* CVXGEN is Copyright (C) 2006-2011 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2011 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */

#ifndef SOLVER_H
#define SOLVER_H

/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */

#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif

/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */

#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif

typedef struct Params_t {
  float xd[4];
  float G[16];
  float A[32];
  float b[8];

} Params;

typedef struct Vars_t {
  float *x; /* 4 rows. */

} Vars;

typedef struct Workspace_t {
  float h[8];
  float s_inv[8];
  float s_inv_z[8];
  float *b;

  float q[4];
  float rhs[20];
  float x[20];
  float *s;
  float *z;
  float *y;
  float lhs_aff[20];
  float lhs_cc[20];
  float buffer[20];
  float buffer2[20];

  float KKT[66];
  float L[46];
  float d[20];
  float v[20];
  float d_inv[20];

  float gap;
  float optval;

  float ineq_resid_squared;
  float eq_resid_squared;

  float block_33[1];

  /* Pre-op symbols. */
  float quad_342101520384[1];

  int converged;
} Workspace;

typedef struct Settings_t {
  float resid_tol;
  float eps;
  int max_iters;
  int refine_steps;

  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  float s_init;
  float z_init;

  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;

  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  float kkt_reg;
} Settings;

extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;

/* Function definitions in /home/jem/olsr/releases/20110330074202/lib/olsr.extra/qp_solver/solver.c: */
float eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexing(void);
void set_start(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(float *target, float *var);
float calc_ineq_resid_squared(void);
float calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in /home/jem/olsr/releases/20110330074202/lib/olsr.extra/qp_solver/matrix_support.c: */
void multbymA(float *lhs, float *rhs);
void multbymAT(float *lhs, float *rhs);
void multbymG(float *lhs, float *rhs);
void multbymGT(float *lhs, float *rhs);
void multbyP(float *lhs, float *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in /home/jem/olsr/releases/20110330074202/lib/olsr.extra/qp_solver/ldl.c: */
void ldl_solve(float *target, float *var);
void ldl_factor(void);
float check_factorization(void);
void matrix_multiply(float *result, float *source);
float check_residual(float *target, float *multiplicand);
void fill_KKT(void);

/* Function definitions in /home/jem/olsr/releases/20110330074202/lib/olsr.extra/qp_solver/util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, float *A, int m, int n, int sparse);
float unif(float lower, float upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
float randn(void);
void reset_rand(void);

/* Function definitions in /home/jem/olsr/releases/20110330074202/lib/olsr.extra/qp_solver/testsolver.c: */
/*int main(int argc, char **argv);
void load_default_data(void);*/

#endif
