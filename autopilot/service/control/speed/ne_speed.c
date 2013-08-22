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
  
 North-East Speed Controller Implementation

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "ne_speed.h"

#include <threadsafe_types.h>
#include <opcd_interface.h>
#include <util.h>

#include "../../util/math/conv.h"
#include "../../util/math/vec2.h"
#include "../util/pid.h"


static tsfloat_t angle_max;
static tsfloat_t speed_p;
static tsfloat_t speed_i;
static tsfloat_t speed_i_max;

static pid_controller_t controllers[2];


void ne_speed_ctrl_init(void)
{
   ASSERT_ONCE();

   /* load parameters: */
   opcd_param_t params[] =
   {
      {"angle_max", &angle_max.value},
      {"p", &speed_p.value},
      {"i", &speed_i.value},
      {"i_max", &speed_i_max.value},
      OPCD_PARAMS_END
   };
   opcd_params_apply("controllers.ne_speed.", params);
   
   /* initialize controllers: */
   FOR_EACH(i, controllers)
   {
      pid_init(&controllers[i], &speed_p, &speed_i, NULL, &speed_i_max);
   }
}


void ne_speed_ctrl_reset(void)
{
   FOR_EACH(i, controllers)
   {
      pid_reset(&controllers[i]);
   }
}


void ne_speed_ctrl_run(vec2_t *pitch_roll_ctrl, const vec2_t *setp, const float dt, const vec2_t *speed, float yaw)
{
   vec2_t ne_ctrl;
   /* run global speed controllers: */
   FOR_EACH(i, controllers)
   {
      float error = setp->vec[i] - speed->vec[i];
      ne_ctrl.vec[i] = -pid_control(&controllers[i], error, 0.0, dt);
   }
   
   /* rotate global speed feedback into local coordinates: */
   vec2_t _pitch_roll_ctrl;
   vec2_world_to_body(&_pitch_roll_ctrl, &ne_ctrl, yaw);
   
   /* limit speed controller output: */
   FOR_EACH(i, controllers)
   {
      pitch_roll_ctrl->vec[i] = sym_limit(_pitch_roll_ctrl.vec[i], deg2rad(tsfloat_get(&angle_max)));
   } 
}

