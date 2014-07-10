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
  
 R/C Sticks Config/Util Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <math.h>

#include <util.h>
#include <threadsafe_types.h>
#include <opcd_interface.h>

#include "../util/math/conv.h"
#include "../util/math/linfunc.h"
#include "../util/logger/logger.h"

#include "sticks.h"


static tsfloat_t pitch_roll_speed_max;
static tsfloat_t pitch_roll_angle_max;
static tsfloat_t vert_speed_max;
static tsfloat_t vert_deadzone;
static tsfloat_t horiz_speed_max;
static tsfloat_t horiz_deadzone;
static tsfloat_t yaw_speed_max;
static tsfloat_t gas_acc_max;
static tsfloat_t rotation;



float sticks_pitch_roll_speed_max(void)
{
   return deg2rad(tsfloat_get(&pitch_roll_speed_max));
}


float sticks_pitch_roll_angle_max(void)
{
   return deg2rad(tsfloat_get(&pitch_roll_angle_max));
}


float sticks_vert_speed_max(void)
{
   return tsfloat_get(&vert_speed_max);
}


float sticks_vert_deadzone(void)
{
   return tsfloat_get(&vert_deadzone);
}


float sticks_horiz_speed_max(void)
{
   return tsfloat_get(&horiz_speed_max);
}


float sticks_horiz_deadzone(void)
{
   return tsfloat_get(&horiz_deadzone);
}


float sticks_yaw_speed_max(void)
{
   return deg2rad(tsfloat_get(&yaw_speed_max));
}


float sticks_gas_acc_max(void)
{
   return tsfloat_get(&gas_acc_max);
}


float sticks_rotation(void)
{
   return deg2rad(tsfloat_get(&rotation));
}


void sticks_init(void)
{
   ASSERT_ONCE();
   LOG(LL_INFO, "initializing sticks configuration");

   opcd_param_t params[] =
   {
      {"pitch_roll_speed_max", &pitch_roll_speed_max},
      {"pitch_roll_angle_max", &pitch_roll_angle_max},
      {"vert_speed_max", &vert_speed_max},
      {"vert_deadzone", &vert_deadzone},
      {"horiz_speed_max", &horiz_speed_max},
      {"horiz_deadzone", &horiz_deadzone},
      {"yaw_speed_max", &yaw_speed_max},
      {"gas_acc_max", &gas_acc_max},
      {"rotation", &rotation},
      OPCD_PARAMS_END
   };
   opcd_params_apply("sticks.", params);
   
   LOG(LL_DEBUG, "pitch_roll_speed_max: %.2f deg/s", rad2deg(sticks_pitch_roll_speed_max()));
   LOG(LL_DEBUG, "pitch_roll_angle_max: %.2f deg", rad2deg(sticks_pitch_roll_angle_max()));
   LOG(LL_DEBUG, "vert_speed_max: %.2f m/s", sticks_vert_speed_max());
   LOG(LL_DEBUG, "vert_deadzone: %.2f", sticks_vert_deadzone());
   LOG(LL_DEBUG, "horiz_speed_max: %.2f m/s", sticks_horiz_speed_max());
   LOG(LL_DEBUG, "horiz_deadzone: %.2f", sticks_horiz_deadzone());
   LOG(LL_DEBUG, "yaw_speed_max: %.2f deg/s", rad2deg(sticks_yaw_speed_max()));
   LOG(LL_DEBUG, "gas_acc_max: %.2f m/s^2", sticks_gas_acc_max());
   LOG(LL_DEBUG, "rotation: %.2f deg", rad2deg(sticks_rotation()));
}


float stick_dz(float g, float d)
{
   float dz_l = -d / 2.0;
   float dz_r = d / 2.0;
   linfunc_t left, right;
   vec2_t pr1;
   vec2_set(&pr1, 0.5f, 0.5f);
   
   vec2_t pr2;
   vec2_set(&pr2, dz_r, 0.0f);
   linfunc_init_points(&right, &pr1, &pr2);
   
   vec2_t pl1;
   vec2_set(&pl1, -0.5f, -0.5f);
   
   vec2_t pl2;
   vec2_set(&pl2, dz_l, 0.0f);
   linfunc_init_points(&left, &pl1, &pl2);
   
   if (g < dz_l)
   {
      return linfunc_calc(&left, g);
   }
   else if (g > dz_r)
   {
      return linfunc_calc(&right, g);
   }
   else
   {
     return 0.0f;
   }
}

