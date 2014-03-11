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
  
 Inverse Coupling Matrix Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <errno.h>

#include <util.h>

#include "inv_coupling.h"


void inv_coupling_init(inv_coupling_t *inv_coupling, const size_t n_motors, const float *init)
{
   ASSERT_ONCE();

   /* allocate inverse coupling matrix :*/
   inv_coupling->matrix = m_get(n_motors, 4);
   ASSERT_NOT_NULL(inv_coupling->matrix);

   /* initialize inverse coupling matrix: */
   FOR_N(i, n_motors)
   {
      FOR_N(j, 4)
      {
         inv_coupling->matrix->me[i][j] = init[i * 4 + j];
      }
   }

   /* allocate input vector: */
   inv_coupling->in = v_get(4);
   ASSERT_NOT_NULL(inv_coupling->in);
   
   /* allocate output vector: */
   inv_coupling->out = v_get(n_motors);
   ASSERT_NOT_NULL(inv_coupling->out);
   
   /* copy motor count: */
   inv_coupling->n_motors = n_motors;
}


void inv_coupling_calc(const inv_coupling_t *inv_coupling, float *out, const float *in)
{
   /* copy data into input vector: */
   FOR_N(i, 4)
   {
      inv_coupling->in->ve[i] = in[i];
   }

   /* matrix-vector multiplication: */
   mv_mlt(inv_coupling->matrix, inv_coupling->in, inv_coupling->out);

   /* copy result of computation into output vector: */
   FOR_N(i, inv_coupling->n_motors)
   {
      out[i] = inv_coupling->out->ve[i];
   }
}

