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
  
 XY Speed Controller Implementation

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "xy_speed.h"

#include <math.h>
#include <util.h>
#include <threadsafe_types.h>
#include <opcd_interface.h>

#include "../../util/math/conv.h"
#include "../../util/math/vec2.h"


static tsfloat_t p;
static tsfloat_t angle_max;


void xy_speed_ctrl_init(void)
{
   ASSERT_ONCE();
   opcd_param_t params[] =
   {
      {"p", &p},
      {"angle_max", &angle_max},
      OPCD_PARAMS_END
   };
   opcd_params_apply("controllers.xy_speed.", params);
}


void xy_speed_ctrl_run(vec2_t *control, vec2_t *speed_setpoint, vec2_t *speed, float yaw)
{
   /* calculate 2d speed error: */
   vec2_t speed_err;
   vec2_sub(&speed_err, speed_setpoint, speed);
   
   /* calculate 2d speed feedback (angle): */
   vec2_t world_thrust;
   vec2_scale(&world_thrust, &speed_err, tsfloat_get(&p));

   /* rotate global speed feedback into local control primitives: */
   vec2_rotate(control, &world_thrust, yaw + M_PI / 2);
   float a_max = deg2rad(tsfloat_get(&angle_max));
   control->vec[0] = -sym_limit(control->vec[0], a_max);
   control->vec[1] = sym_limit(control->vec[1], a_max);
}

