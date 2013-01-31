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
  
 Actuator Characteristic - Implementation

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
#include "util.h"


#include "ac.h"


void ac_init(ac_t *ac, float min, float max, float v_min, float v_max,
             float f_c, size_t n_motors, acf_t func, float off_val)
{
   assert(ac);
   assert(max > min);
   assert(v_max > v_min);
   assert(f_c > 0.0f);
   assert(n_motors > 0);
   assert(func);
   ac->min = min;
   ac->max = max;
   ac->v_min = v_min;
   ac->v_max = v_max;
   ac->f_c = f_c;
   ac->n_motors = n_motors;
   ac->func = func;
   ac->off_val = off_val;
}


int ac_calc(float *out, const ac_t *ac, const int enabled,
            const float voltage, const float *rpm_sq)
{
   assert(out);
   assert(ac);
   assert(rpm_sq);
   int saturated = 0;
   if (!enabled)
   {
      /* actuators disabled */
      saturated = 1;
      FOR_N(i, ac->n_motors)
      {
         out[i] = ac->off_val;
      }
   }
   else
   {
      /* actuators enabled */
      const float _voltage = limit(voltage, ac->v_min, ac->v_max);
      FOR_N(i, ac->n_motors)
      {
         float val = ac->func(rpm_sq[i] * ac->f_c, _voltage);
         if (val < ac->min)
         {
            out[i] = ac->min;
            saturated = 1;
         }
         else if (val > ac->max)
         {
            out[i] = ac->max;
            saturated = 1;
         }
         else
         {
            out[i] = val;
         }
      }
   }
   return saturated;
}

