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

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

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


int att_thrust_calc(vec2_t *pr_angles, float *thrust, /* output: pitch/roll angles + thrust */
                    vec3_t *f_neu,   /* input of p,r,d forces */
                    float yaw, /* yaw orientation */
                    float thrust_max, /* maximum thrust force */
                    int update_f_neu  /* if true, f_neu is updated if thrust_max has been exceeded */)
{
   int constrained = 0;
   float n = f_neu->vec[0];
   float e = f_neu->vec[1];
   float u = f_neu->vec[2];
   assert(thrust_max >= 0);

   if (u < 2.0)
   {
      u = 2.0f;
      constrained = 1;
   }

   /* compute unconstrained thrust: */
   *thrust = sqrtf(n * n + e * e + u * u);
   
   /* check if computed thrust exceeds maximal thrust: */
   if (*thrust > thrust_max)
   {
      /* compute constrained forces: */
      if (u > thrust_max)
      {
         /* thrust_max is exceeded by d: */
         n = 0.0;
         e = 0.0;
         u = thrust_max;
      }
      else /* u <= thrust_max: */
      {
         /* scale down n and e to meet thrust limit: */
         float u_remain = sqrtf((thrust_max * thrust_max) - u * u);
         float ne_len = sqrtf(n * n + e * e);
         float factor = u_remain / ne_len;
         n = n * factor;
         e = e * factor;
      }
      /* compute constrained thrust: */
      *thrust = sqrtf(n * n + e * e + u * u);
      constrained = 1;
   }
   
   /* update f_neu, if requested: */
   if (update_f_neu)
   {
      f_neu->vec[0] = n;
      f_neu->vec[1] = e;
      f_neu->vec[2] = u;
   }

   /* compute resulting angles: */
   vec2_t ne_angles;
   if (u > 0.0f)
   {
      ne_angles.x = atan2(n, u);
      ne_angles.y = atan2(e, u);
   }
   else
   {
      ne_angles.x = 0.0f;
      ne_angles.y = 0.0f;
   }

   /* rotate global horizonzal thrust to device-local: */
   vec2_rotate(pr_angles, &ne_angles, -yaw);
   return constrained;
}

