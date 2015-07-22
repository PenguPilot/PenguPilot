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
  
 PID Controller Implementation

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include "pid.h"
#include <util.h>


void pid_init(pid_controller_t *controller, tsfloat_t *p, tsfloat_t *i, tsfloat_t *d, tsfloat_t *d_filt, tsfloat_t *max_sum_error)
{
   ASSERT_NOT_NULL(p);
   controller->p = p;
   controller->i = i; /* might be NULL .. */
   if (i != NULL)
   {
      /* max_sum_error is optional */
      controller->max_sum_error = max_sum_error;
      tsfloat_init(&controller->sum_error, 0.0f);
   }
   controller->d = d; /* ... this one as well */
   controller->prev_error = 0.0f;
   controller->d_state = 0.0f;
   controller->d_filt = d_filt;
}


float pid_control(pid_controller_t *controller, const float error, const float speed, const float dt)
{
   float val = tsfloat_get(controller->p) * error;
   if (controller->i != NULL)
   {
      float sum_error = tsfloat_get(&controller->sum_error);
      if (controller->i_enabled)
         sum_error += error * dt;
      if (controller->max_sum_error != NULL)
         sum_error = sym_limit(sum_error, tsfloat_get(controller->max_sum_error));
      val += tsfloat_get(controller->i) * sum_error;
      tsfloat_set(&controller->sum_error, sum_error);
   }
   if (controller->d != NULL)
   {
      float d_filt = tsfloat_get(controller->d_filt);
      float new_val = (speed == 0.0f) ? (error - controller->prev_error) / dt : -speed;
      controller->d_state = d_filt * new_val + (1.0f - d_filt) * controller->d_state;
      val += tsfloat_get(controller->d) * controller->d_state;
   }
   controller->prev_error = error;
   return val;
}


void pid_reset(pid_controller_t *controller)
{
   tsfloat_set(&controller->sum_error, 0.0f);
}

