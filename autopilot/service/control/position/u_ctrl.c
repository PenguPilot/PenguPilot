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
  
 up pos controller implementation
 
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

#include "u_ctrl.h"
#include "../util/pid.h"
#include "../../filters/filter.h"


static pid_controller_t ctrl;


static tsfloat_t pos_p;
static tsfloat_t pos_d;
static tsfloat_t pos_i;
static tsfloat_t pos_imax;
static tsfloat_t lpfg;
static Filter1 filter;


float u_ctrl_step(float *err, const float setpoint, const float pos, const float speed, const float dt)
{
   *err = setpoint - pos;
   return pid_control(&ctrl, *err, speed, dt);
}


void u_ctrl_init(const float dt)
{
   ASSERT_ONCE();
   
   opcd_param_t params[] =
   {
      {"p", &pos_p},
      {"d", &pos_d},
      {"i", &pos_i},
      {"imax", &pos_imax},
      {"lpfg", &lpfg},
      OPCD_PARAMS_END
   };
   opcd_params_apply("controllers.u_pos.", params);

   pid_init(&ctrl, &pos_p, &pos_i, &pos_d, &pos_imax);
   filter1_lp_init(&filter, tsfloat_get(&lpfg), dt, 1);
}


void u_ctrl_reset(void)
{
   pid_reset(&ctrl);
}

