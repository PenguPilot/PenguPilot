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
  
 Yaw Controller Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <malloc.h>
#include <math.h>

#include <util.h>
#include <opcd_interface.h>
#include <threadsafe_types.h>

#include "yaw_ctrl.h"
#include "../util/pid.h"
#include "../../util/logger/logger.h"


static pid_controller_t controller;

/* configurable parameters: */
static tsfloat_t p;
static tsfloat_t i;
static tsfloat_t i_max;
static tsfloat_t d;
static tsfloat_t pid_lim;


static float circle_err(float pos, float dest)
{
   float err;
   if ((dest < pos) && (pos - dest < M_PI))
   {
      err = pos - dest;
   }
   else if ((dest < pos) && (pos - dest >= M_PI))
   {
      err = -(2.0f * (float)M_PI - (pos - dest));
   }
   else if ((dest > pos) && (dest - pos < M_PI))
   {
      err = -(dest - pos);
   }
   else if ((dest > pos) && (dest - pos >= M_PI))
   {
      err = 2.0f * (float)M_PI - (dest - pos);
   }
   else
   {
      err = 0.0f;
   }
   return err;
}


void yaw_ctrl_init(void)
{
   ASSERT_ONCE();
   opcd_param_t params[] =
   {
      {"p", &p.value},
      {"i", &i.value},
      {"i_max", &i_max.value},
      {"d", &d.value},
      {"pid_lim", &pid_lim.value},
      OPCD_PARAMS_END
   };
   opcd_params_apply("controllers.yaw.", params);
   pid_init(&controller, &p, &i, &d, &i_max);
}


float yaw_ctrl_step(float *err_out, float setpoint, float yaw, float _speed, float dt)
{
   float err;
   float yaw_ctrl;
   err = circle_err(setpoint, yaw);
   yaw_ctrl = pid_control(&controller, err, _speed, dt);
   *err_out = err;
   return sym_limit(yaw_ctrl, tsfloat_get(&pid_lim));
}


void yaw_ctrl_reset(void)
{
   pid_reset(&controller);
}

