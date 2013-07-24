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
  
 z speed controller implementation
 
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

#include "z_speed.h"
#include "../util/pid.h"


static pid_controller_t ctrl;


static float z_neutral_gas;
static tsfloat_t speed_p;
static tsfloat_t speed_i;
static tsfloat_t speed_imax;
static tsfloat_t speed_d;


float z_speed_step(float setpoint, float speed, float dt)
{   
   float err = setpoint - speed;
   return z_neutral_gas + pid_control(&ctrl, err, -speed, dt);
}


void z_speed_init(float neutral_gas)
{
   ASSERT_ONCE();
   
   opcd_param_t params[] =
   {
      {"speed_p", &speed_p},
      {"speed_i", &speed_i},
      {"speed_d", &speed_d},
      {"speed_imax", &speed_imax},
      OPCD_PARAMS_END
   };
   opcd_params_apply("controllers.z_speed.", params);

   pid_init(&ctrl, &speed_p, &speed_i, &speed_d, &speed_imax);
   z_neutral_gas = neutral_gas;
}


void z_speed_reset(void)
{
   pid_reset(&ctrl);
}

