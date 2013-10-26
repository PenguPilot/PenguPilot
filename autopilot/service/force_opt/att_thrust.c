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
  
 Attitude and Thrust Computation Implementation

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include <assert.h>
#include <math.h>
#include <string.h>


#include "att_thrust.h"


int att_thrust_calc(vec2_t *ne_out, float *thrust, /* output: angle north, angle east, thrust */
                    vec3_t *f_ned,   /* input of n,e,d forces */
                    float thrust_max, /* maximum thrust force */
                    int update_f_ned  /* if true, f_ned is updated if thrust_max has been exceeded */)
{
   int constrained = 0;
   float n = f_ned->vec[0];
   float e = f_ned->vec[1];
   float d = f_ned->vec[2];
   assert(thrust_max >= 0);
   if (d < 0.0)
      d = 0.0;
   
   /* compute unconstrained thrust: */
   *thrust = sqrtf(n * n + e * e + d * d);
   
   /* check if computed thrust exceeds maximal thrust: */
   if (*thrust > thrust_max)
   {
      /* compute constrained forces: */
      if (d >= thrust_max)
      {
         /* thrust_max is exceeded by d: */
         n = 0.0;
         e = 0.0;
         d = thrust_max;
      }
      else /* d < thrust_max: */
      {
         /* scale down n and e to meet thrust limit: */
         float d_remain = sqrtf((thrust_max * thrust_max) - d * d);
         float ne_len = sqrtf(n * n + e * e);
         float factor = d_remain / ne_len;
         n = n * factor;
         e = e * factor;
      }
      /* compute constrained thrust: */
      *thrust = sqrtf(n * n + e * e + d * d);
      constrained = 1;
   }

   /* update f_ned, if requested: */
   if (update_f_ned)
   {
      f_ned->vec[0] = n;
      f_ned->vec[1] = e;
      f_ned->vec[2] = d;
   }

   /* set output angles and thrust: */
   ne_out->vec[0] = atan(n / d);
   ne_out->vec[1] = atan(e / d);

   return constrained;
}

