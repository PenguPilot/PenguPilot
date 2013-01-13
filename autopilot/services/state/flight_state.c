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
  
 Flight State Tracking

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <util.h>
#include <malloc.h>

#include "../filters/sliding_var.h"
#include "flight_state.h"



static sliding_var_t *vars;
static size_t dim;
static size_t init_cnt;
static size_t wnd;
static float fly_tresh;
static float crash_tresh;
static float min_ground_z;
static float hyst;
static int hyst_cnt = 0;

static flight_state_t state = FS_STANDING;


void flight_state_init(size_t window, size_t hysteresis, float fly_treshold, float crash_treshold, float _min_ground_z)
{
   ASSERT_ONCE();
   min_ground_z = _min_ground_z;
   hyst = hysteresis;
   fly_tresh = fly_treshold;
   crash_tresh = crash_treshold;
   wnd = window;
   dim = 3;
   vars = malloc(dim * sizeof(sliding_var_t));
   ASSERT_NOT_NULL(vars);
   FOR_N(i, dim)
   {
      sliding_var_init(&vars[i], window, 0.0f);   
   }
}


flight_state_t flight_state_update(float acc[3], float ground_z)
{
   /* perfom signal processing: */
   float sum = 0;
   FOR_N(i, dim)
   {
      float var = sliding_var_calc(&vars[i], acc[i]);
      sum += var;
   }
   sum /= dim;

   /* signal/hysteresis-based state identification: */
   if (sum > fly_tresh)
   {
      /* flying -> flying / standing -> flying */
      state = FS_FLYING;
      hyst_cnt = 0;
   }
   else
   {
      if (state == FS_FLYING && ground_z < min_ground_z)
      {
         /* flying -> standing */
         if (hyst_cnt++ == hyst)
         {
            state = FS_STANDING;
         }
      }
   }
   return state;
}

