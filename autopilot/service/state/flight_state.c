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

 The module integrates two sensor inputs:
 - accelerometer variance
 - ultrasonic sensor

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

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
static size_t wnd;
static float tresh;
static float hyst;
static int fly_hyst_cnt = 0;
static int stand_hyst_cnt = 0;

static bool flying = false;


void flight_state_init(size_t window, size_t hysteresis, float treshold)
{
   ASSERT_ONCE();
   hyst = hysteresis;
   tresh = treshold;
   wnd = window;
   dim = 2;
   vars = malloc(dim * sizeof(sliding_var_t));
   ASSERT_NOT_NULL(vars);

   FOR_N(i, dim)
      sliding_var_init(&vars[i], window, 0.0f);   
}


bool flight_state_update(float acc[3])
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
   if (sum > tresh)
   {
      stand_hyst_cnt = 0;
      /* flying -> flying / standing -> flying */
      if (fly_hyst_cnt++ == hyst)
         flying = true;
   }
   else
   {
      fly_hyst_cnt = 0;
      /* flying -> standing */
      if (stand_hyst_cnt++ == hyst)
         flying = false;
   }
   return flying;
}

