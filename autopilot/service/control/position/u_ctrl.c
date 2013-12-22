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
  
 Up Position Controller Implementation
 
 Copyright (C) 2010 Tobias Simon, Ilmenau University of Technology

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


static pid_controller_t controller;


/* initialized by "init" parameter: */
static float u_neutral_gas;

/* following thread-safe variables are initialized by pid_init: */
static tsfloat_t speed_p;
static tsfloat_t speed_i;
static tsfloat_t speed_imax;
static tsfloat_t speed_d;

/* following thread-safe variables are initialized by OPCD: */
static tsfloat_t init_setpoint;
static tsint_t init_mode_is_ground;

/* following thread-safe variables are initialized by u_ctrl_init: */
static tsfloat_t setpoint;
static tsint_t mode_is_ground;



/* returns the desired vertical speed given a error */
static float speed_func(float err)
{
   return sym_limit(err, 1.0f) * 0.6;
}


void u_ctrl_set_setpoint(u_setpoint_t u_setpoint)
{
   tsfloat_set(&setpoint, u_setpoint.val);
   tsint_set(&mode_is_ground, u_setpoint.mode_is_ground);
}


float u_ctrl_get_setpoint(void)
{
   return tsfloat_get(&setpoint);
}


int u_ctrl_mode_is_ground(void)
{
   return tsint_get(&mode_is_ground);
}


/*
 * perform controller step
 * parameters ground_u_pos, u_pos, float speed: need to be filtered (e.g. kalman)
 * returns vertical controller output
 */
float u_ctrl_step(float *u_err, float ground_u_pos, float u_pos, float speed, float dt)
{   
   float _u_err;
   if (tsint_get(&mode_is_ground))
   {
      _u_err = tsfloat_get(&setpoint) - ground_u_pos;
   }
   else
   {
      _u_err = tsfloat_get(&setpoint) - u_pos;
   }
   float spd_sp = speed_func(_u_err);
   float spd_err = spd_sp - speed;
   float val;
   val = u_neutral_gas + pid_control(&controller, _u_err, -speed, dt);
   *u_err = _u_err;
   return val;
}


void u_ctrl_init(float neutral_gas)
{
   ASSERT_ONCE();
   
   opcd_param_t params[] =
   {
      {"speed_p", &speed_p},
      {"speed_i", &speed_i},
      {"speed_d", &speed_d},
      {"speed_imax", &speed_imax},
      {"init_setpoint", &init_setpoint},
      {"init_mode_is_ground", &init_mode_is_ground},
      OPCD_PARAMS_END
   };
   opcd_params_apply("controllers.altitude.", params);

   tsfloat_init(&setpoint, tsfloat_get(&init_setpoint));
   tsint_init(&mode_is_ground, tsint_get(&init_mode_is_ground));
   pid_init(&controller, &speed_p, &speed_i, &speed_d, &speed_imax);
   u_neutral_gas = neutral_gas;
}


void u_ctrl_reset(void)
{
   pid_reset(&controller);
}

