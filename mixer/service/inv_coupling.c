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

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <errno.h>
#include <string.h>

#include <util.h>
#include <simple_thread.h>
#include <threadsafe_types.h>
#include <motors.h>
#include <math/vec.h>
#include <math/mat.h>


#include "inv_coupling.h"


VEC_DECL(4);


struct
{
   size_t n_motors;
   mat_t matrix;
   vec4_t in;
   vec_t out;
}
inv_coupling;


void inv_coupling_init(const size_t n_motors, const float mixer[FORCES_AND_MOMENTS][MAX_MOTORS])
{
   ASSERT_ONCE();
   
   /* allocate inverse coupling matrix :*/
   mat_alloc(&inv_coupling.matrix, n_motors, FORCES_AND_MOMENTS);

   /* initialize inverse coupling matrix: */
   FOR_N(i, n_motors)
   {
      FOR_N(j, FORCES_AND_MOMENTS)
      {
         inv_coupling.matrix.ve[i * FORCES_AND_MOMENTS + j] = mixer[j][i];
      }
   }

   /* allocate input vector: */
   vec4_init(&inv_coupling.in);
   
   /* allocate output vector: */
   vec_alloc(&inv_coupling.out, n_motors);
   
   /* copy motor count: */
   inv_coupling.n_motors = n_motors;
}


void inv_coupling_calc(float *out, const float *in)
{
   /* copy data into input vector: */
   FOR_N(i, 4)
   {
      inv_coupling.in.ve[i] = in[i];
   }

   /* matrix-vector multiplication: */
   mat_vec_mul(&inv_coupling.out, &inv_coupling.matrix, &inv_coupling.in);

   /* copy result of computation into output vector: */
   FOR_N(i, inv_coupling.n_motors)
   {
      out[i] = inv_coupling.out.ve[i];
   }
}

