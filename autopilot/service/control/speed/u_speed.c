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
  
 up speed controller implementation
 
 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <util.h>
#include <opcd_interface.h>
#include <threadsafe_types.h>

#include "u_speed.h"
#include "../util/pid.h"


static pid_controller_t ctrl;


static tsfloat_t speed_p;
static tsfloat_t speed_i;
static tsfloat_t speed_imax;


float u_speed_ctrl_step(float *err, const float setpoint, const float pos, const float dt)
{   
   *err = setpoint - pos;
   return pid_control(&ctrl, *err, 0.0f, dt);
}


void u_speed_ctrl_init(void)
{
   ASSERT_ONCE();
   
   opcd_param_t params[] =
   {
      {"p", &speed_p},
      {"i", &speed_i},
      {"imax", &speed_imax},
      OPCD_PARAMS_END
   };
   opcd_params_apply("controllers.u_speed.", params);
   pid_init(&ctrl, &speed_p, &speed_i, NULL, &speed_imax);
}


void u_speed_ctrl_reset(void)
{
   pid_reset(&ctrl);
}

