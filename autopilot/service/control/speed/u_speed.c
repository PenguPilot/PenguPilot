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
 
 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

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


static float u_neutral_gas;
static tsfloat_t speed_p;
static tsfloat_t speed_d;
static tsfloat_t speed_i;
static tsfloat_t speed_imax;


float u_speed_step(float setpoint, float pos, float speed, float dt)
{   
   float err = setpoint - pos;
   return u_neutral_gas + pid_control(&ctrl, err, speed, dt);
}


void u_speed_init(float neutral_gas)
{
   ASSERT_ONCE();
   
   opcd_param_t params[] =
   {
      {"p", &speed_p},
      {"d", &speed_d},
      {"i", &speed_i},
      {"imax", &speed_imax},
      OPCD_PARAMS_END
   };
   opcd_params_apply("controllers.u_speed.", params);

   pid_init(&ctrl, &speed_p, &speed_i, &speed_d, &speed_imax);
   u_neutral_gas = neutral_gas;
}


void u_speed_reset(void)
{
   pid_reset(&ctrl);
}

