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
  
 Force Optimizer Implementation

 Copyright (C) 2012 Alexander Barth, Ilmenau University of Technology
 Copyright (C) 2012 Benjamin Jahn, Ilmenau University of Technology
 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "cvxgen/solver.h"


Vars vars;
Params params;
Workspace work;
Settings settings;


void force_opt_init(float imtx1, float imtx2, float imtx3, float rpm_square_min, float rpm_square_max)
{
  set_defaults();
  setup_indexing();
  
  /* G-Matrix */
  params.G[0] = 1e-4;
  params.G[4] = 0;
  params.G[8] = 0;
  params.G[12] = 0;

  params.G[1] = 0;
  params.G[5] = 1.0;
  params.G[9] = 0;
  params.G[13] = 0;

  params.G[2] = 0;
  params.G[6] = 0;
  params.G[10] = 1.0;
  params.G[14] = 0;

  params.G[3] = 0;
  params.G[7] = 0;
  params.G[11] = 0;
  params.G[15] = 2.0;

  /* A-Matrix */
  params.A[0] = (float)((double)imtx1)/((double)((double)rpm_square_max));
  params.A[8] = 0.0;
  params.A[16] = -(float)((double)imtx2)/((double)rpm_square_max);
  params.A[24] = (float)((double)imtx3)/((double)rpm_square_max);

  params.A[1] = (float)((double)imtx1)/((double)rpm_square_max);
  params.A[9] = 0.0;
  params.A[17] = (float)((double)imtx2)/((double)rpm_square_max);
  params.A[25] = (float)((double)imtx3)/((double)rpm_square_max);

  params.A[2] = (float)((double)imtx1)/((double)rpm_square_max);
  params.A[10] = -(float)((double)imtx2)/((double)rpm_square_max);
  params.A[18] = 0.0;;
  params.A[26] = -((float)(double)imtx3)/((double)rpm_square_max);

  params.A[3] = (float)((double)imtx1)/((double)rpm_square_max);
  params.A[11] = (float)((double)imtx2)/((double)rpm_square_max);
  params.A[19] = 0.0;
  params.A[27] = -(float)((double)imtx3)/((double)rpm_square_max);

  params.A[4]  = -params.A[0];
  params.A[12] = -params.A[8];
  params.A[20] = -params.A[16];
  params.A[28] = -params.A[24];

  params.A[5]  = -params.A[1];
  params.A[13] = -params.A[9];
  params.A[21] = -params.A[17];
  params.A[29] = -params.A[25];

  params.A[6]  = -params.A[2];
  params.A[14] = -params.A[10];
  params.A[22] = -params.A[18];
  params.A[30] = -params.A[26];

  params.A[7]  = -params.A[3];
  params.A[15] = -params.A[11];
  params.A[23] = -params.A[19];
  params.A[31] = -params.A[27];

  params.b[0] = 1.0;
  params.b[1] = 1.0;
  params.b[2] = 1.0;
  params.b[3] = 1.0;
  params.b[4] = -(float)((double)rpm_square_min)/((double)rpm_square_max);
  params.b[5] = -(float)((double)rpm_square_min)/((double)rpm_square_max);
  params.b[6] = -(float)((double)rpm_square_min)/((double)rpm_square_max);
  params.b[7] = -(float)((double)rpm_square_min)/((double)rpm_square_max);

  settings.verbose = 0;  // disable output of solver progress.
  settings.max_iters = 10;  // reduce the maximum iteration count, from 25.
  settings.eps = 1e-6;;  // reduce the required objective tolerance, from 1e-6.
  settings.resid_tol = 1e-6;  // reduce the required residual tolerances, from 1e-4.
  settings.better_start = 1;
}


void force_opt_run(float forces[4])
{
  params.xd[0] = forces[0];
  params.xd[1] = forces[1];
  params.xd[2] = forces[2];
  params.xd[3] = forces[3];

  solve();

  forces[0] = vars.x[0];
  forces[1] = vars.x[1];
  forces[2] = vars.x[2];
  forces[3] = vars.x[3];
}

