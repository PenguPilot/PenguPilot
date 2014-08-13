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
#include <linfunc.h>

#include "../util/math/conv.h"
#include "../util/logger/logger.h"

#include "sticks.h"


static tsfloat_t pitch_roll_speed_max;
static tsfloat_t pitch_roll_angle_max;
static tsfloat_t vert_speed_max;
static tsfloat_t vert_deadzone;
static tsfloat_t horiz_speed_max;
static tsfloat_t horiz_deadzone;
static tsfloat_t yaw_speed_max;
static tsfloat_t yaw_deadzone;
static tsfloat_t gas_acc_max;
static tsfloat_t rotation;
static tsfloat_t expo;


static float sticks_pitch_roll_speed_max(void)
{
   return deg2rad(tsfloat_get(&pitch_roll_speed_max));
}


static float sticks_pitch_roll_angle_max(void)
{
   return deg2rad(tsfloat_get(&pitch_roll_angle_max));
}


static float sticks_vert_speed_max(void)
{
   return tsfloat_get(&vert_speed_max);
}


static float sticks_vert_deadzone(void)
{
   return tsfloat_get(&vert_deadzone);
}


static float sticks_horiz_speed_max(void)
{
   return tsfloat_get(&horiz_speed_max);
}


static float sticks_horiz_deadzone(void)
{
   return tsfloat_get(&horiz_deadzone);
}


static float sticks_yaw_speed_max(void)
{
   return deg2rad(tsfloat_get(&yaw_speed_max));
}


static float sticks_yaw_deadzone(void)
{
   return tsfloat_get(&yaw_deadzone);
}


static float sticks_gas_acc_max(void)
{
   return tsfloat_get(&gas_acc_max);
}


float sticks_rotation(void)
{
   return deg2rad(tsfloat_get(&rotation));
}


static float stick_expo(float x)
{
   float base = tsfloat_get(&expo);
   if (base <= 1.0f)
      base = 1.01f;
   float scale = 1.0f / (base - 1.0f);
   if (x >= 0.0f)
      return scale * (powf(base, x) - 1.0f);
   else
      return -scale * (powf(base, -x) - 1.0f);
}


float stick_dz(float g, float d)
{
   float dz_l = -d / 2.0;
   float dz_r = d / 2.0;
   linfunc_t left, right;
   linfunc_init_points(&right, 0.5f, 0.5f, dz_r, 0.0f);
   linfunc_init_points(&left, -0.5f, -0.5f, dz_l, 0.0f);
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
      {"yaw_deadzone", &yaw_deadzone},
      {"gas_acc_max", &gas_acc_max},
      {"rotation", &rotation},
      {"expo", &expo},
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
   LOG(LL_DEBUG, "yaw_deadzone: %.2f", sticks_yaw_deadzone());
   LOG(LL_DEBUG, "gas_acc_max: %.2f m/s^2", sticks_gas_acc_max());
   LOG(LL_DEBUG, "rotation: %.2f deg", rad2deg(sticks_rotation()));
}


float sticks_pitch_roll_speed_func(float stick)
{
   return sticks_pitch_roll_speed_max() * stick_expo(stick);
}


float sticks_pitch_roll_angle_func(float stick)
{
   return sticks_pitch_roll_angle_max() * stick_expo(stick);
}


float sticks_gas_acc_func(float stick)
{
   return sticks_gas_acc_max() * stick_expo(stick);
}


float sticks_gas_speed_func(float stick)
{
   return sticks_vert_speed_max() * stick_expo(stick);
}


void sticks_pitch_roll_gps_speed_func(vec2_t *speed, const vec2_t *sticks)
{
   vec2_t expo_sticks;
   vec2_set(&expo_sticks, stick_expo(sticks->ve[0]), stick_expo(sticks->ve[1]));
   float norm = vec2_norm(&expo_sticks);
   if (norm > sqrt(2.0f))
      vec2_normalize(&expo_sticks, &expo_sticks);
   vec2_scale(speed, &expo_sticks, sticks_horiz_speed_max());
}


bool sticks_pitch_roll_in_deadzone(float pitch, float roll)
{
   return hypot(pitch, roll) < sticks_horiz_deadzone();
}


bool sticks_gas_in_deadzone(float gas)
{
   return fabs(gas) < sticks_vert_deadzone();
}


float sticks_gas_speed_deadzone(float gas)
{
   float vmax = sticks_vert_speed_max();
   return stick_dz(gas, sticks_vert_deadzone()) * vmax;
}


float sticks_yaw_speed_deadzone(float yaw)
{
   return stick_dz(yaw, sticks_yaw_deadzone() * sticks_yaw_speed_max());
}

