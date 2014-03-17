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

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

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


void pid_init(pid_controller_t *controller, tsfloat_t *p, tsfloat_t *i, tsfloat_t *d, tsfloat_t *max_sum_error)
{
   ASSERT_NOT_NULL(p);
   controller->p = p;
   controller->i = i; /* might be NULL .. */
   if (i != NULL)
   {
      ASSERT_NOT_NULL(max_sum_error);
      controller->max_sum_error = max_sum_error;
      tsfloat_init(&controller->sum_error, 0.0f);
   }
   controller->d = d; /* ... this one as well */
}


float pid_control(pid_controller_t *controller, float error, float speed, float dt)
{
   float val = tsfloat_get(controller->p) * error;
   if (controller->i != NULL)
   {
      float sum_error = tsfloat_get(&controller->sum_error);
      sum_error = sym_limit(sum_error + error * dt, tsfloat_get(controller->max_sum_error));
      val += tsfloat_get(controller->i) * sum_error;
      tsfloat_set(&controller->sum_error, sum_error);
   }
   if (controller->d != NULL)
   {
      val += tsfloat_get(controller->d) * (-speed);
   }
   return val;
}


void pid_reset(pid_controller_t *controller)
{
   tsfloat_set(&controller->sum_error, 0.0f);
}

