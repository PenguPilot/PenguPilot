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
  
 Manual Flight Logic Implementation

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

#include <threadsafe_types.h>
#include <opcd_interface.h>
#include <util.h>

#include "man_logic.h"
#include "../hardware/util/calibration.h"
#include "../util/logger/logger.h"
#include "../util/math/conv.h"
#include "../util/math/linfunc.h"
#include "../util/time/interval.h"
#include "../platform/platform.h"
#include "../control/position/navi.h"
#include "../control/position/u_ctrl.h"
#include "../main_loop/control_mode.h"


typedef enum
{
   MAN_SPORT,
   MAN_RELAXED,
   MAN_NOVICE
}
man_mode_t;
man_mode_t last_mode = -1;


static tsfloat_t pitch_roll_speed_max;
static tsfloat_t pitch_roll_angle_max;
static tsfloat_t vert_speed_max;
static tsfloat_t horiz_speed_max;
static tsfloat_t gps_deadzone;
static tsfloat_t gas_deadzone;
static tsfloat_t yaw_speed_max;
static tsfloat_t sticks_rotation;

static bool vert_pos_locked = false;
static bool horiz_pos_locked = true;


void man_logic_init(void)
{
   opcd_param_t params[] =
   {
      {"pitch_roll_speed_max", &pitch_roll_speed_max},
      {"pitch_roll_angle_max", &pitch_roll_angle_max},
      {"vert_speed_max", &vert_speed_max},
      {"horiz_speed_max", &horiz_speed_max},
      {"gps_deadzone", &gps_deadzone},
      {"gas_deadzone", &gas_deadzone},
      {"yaw_speed_max", &yaw_speed_max},
      {"sticks_rotation", &sticks_rotation},
      OPCD_PARAMS_END
   };
   opcd_params_apply("manual_control.", params);
}


static man_mode_t channel_to_man_mode(float sw)
{
   float a = 1.0f / 3.0f;
   float b = 2.0f / 3.0f;

   if (sw <= a)
   {
      return MAN_SPORT;   
   }
   else if (sw > a && sw < b)
   {
      return MAN_RELAXED;
   }
   return MAN_NOVICE;
}


static void handle_mode_update(man_mode_t mode)
{
   if (last_mode != mode)
   {
      LOG(LL_INFO, "switching manual mode to: %d", mode);
      last_mode = mode;   
   }
}


static float stick_dz(float g, float d)
{
   float dz_l = -d / 2.0;
   float dz_r = d / 2.0;
   linfunc_t left, right;
   vec2_t pr1 = {{0.5f, 0.5f}};
   vec2_t pr2 = {{dz_r, 0.0f}};
   linfunc_init_points(&right, &pr1, &pr2);
   vec2_t pl1 = {{-0.5f, -0.5f}};
   vec2_t pl2 = {{dz_l, 0.0f}};
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



static void set_vertical_spd_or_pos(float gas_stick, float u_baro_pos, float u_ultra_pos)
{
   float dz = tsfloat_get(&gas_deadzone);
   if (fabs(gas_stick) > dz || u_ultra_pos < 0.5)
   {
      float vmax = tsfloat_get(&vert_speed_max);
      cm_u_set_spd(stick_dz(gas_stick, dz) * vmax);
      vert_pos_locked = false;
   }
   else if (!vert_pos_locked)
   {
      vert_pos_locked = true;
      if (u_ultra_pos < 5.0)
         cm_u_set_ultra_pos(u_ultra_pos);
      else
         cm_u_set_baro_pos(u_baro_pos);
   }
}


static void set_pitch_roll_rates(float pitch, float roll)
{
   float dz = tsfloat_get(&gps_deadzone);
   float rate_max = deg2rad(tsfloat_get(&pitch_roll_speed_max));
   vec2_t pitch_roll = {{rate_max * stick_dz(pitch, dz), rate_max * stick_dz(roll, dz)}};
   cm_att_set_rates(pitch_roll);
}


static void set_horizontal_spd_or_pos(float pitch, float roll, float yaw, vec2_t *ne_gps_pos, float u_ultra_pos)
{
   if (1) //sqrt(pitch * pitch + roll * roll) > tsfloat_get(&gps_deadzone) || u_ultra_pos < 1.0)
   {
      /* set GPS speed based on sticks input: */
      float vmax_sqrt = sqrt(tsfloat_get(&horiz_speed_max));
      float dz = tsfloat_get(&gps_deadzone);
      vec2_t pitch_roll_spd_sp = {{vmax_sqrt * stick_dz(pitch, dz), vmax_sqrt * stick_dz(roll, dz)}};
      vec2_t ne_spd_sp;
      vec2_rotate(&ne_spd_sp, &pitch_roll_spd_sp, yaw);
      cm_att_set_gps_spd(ne_spd_sp);
      horiz_pos_locked = false;
   }
   else if (!horiz_pos_locked)
   {
      /* lock GPS position until next sticks activity: */
      navi_reset();
      horiz_pos_locked = true;
      cm_att_set_gps_pos(*ne_gps_pos);   
   }
}


void set_att_angles(float pitch, float roll)
{
   float dz = tsfloat_get(&gps_deadzone);
   float angle_max = deg2rad(tsfloat_get(&pitch_roll_angle_max));
   vec2_t pitch_roll = {{angle_max * stick_dz(pitch, dz), angle_max * stick_dz(roll, dz)}};
   cm_att_set_angles(pitch_roll);
}


static void emergency_landing(bool gps_valid, vec2_t *ne_gps_pos, float u_ultra_pos)
{
   cm_u_set_spd(-0.5f);
   if (gps_valid)
      set_horizontal_spd_or_pos(0.0f, 0.0f, 0.0f, ne_gps_pos, u_ultra_pos);
   else
      set_att_angles(0.0f, 0.0f);
   
   if (u_ultra_pos < 0.3f)
      cm_disable_motors_persistent();
}


bool man_logic_run(bool *hard_off, uint16_t sensor_status, bool flying, float channels[MAX_CHANNELS], float yaw, vec2_t *ne_gps_pos, float u_baro_pos, float u_ultra_pos, float f_max, float mass)
{
   vec2_t pr = {{channels[CH_PITCH], channels[CH_ROLL]}};
   vec2_rotate(&pr, &pr, deg2rad(tsfloat_get(&sticks_rotation)));
   float pitch = pr.vec[0];
   float roll = pr.vec[1];

   float yaw_stick = channels[CH_YAW];
   float gas_stick = channels[CH_GAS];
   float sw_l = channels[CH_SWITCH_L];
   float sw_r = channels[CH_SWITCH_R];
   bool gps_valid = (sensor_status & GPS_VALID) ? true : false;

   /*if (!(sensor_status & RC_VALID))
      emergency_landing(gps_valid, ne_gps_pos, u_ultra_pos);
   */

   cm_yaw_set_spd(stick_dz(yaw_stick, 0.075) * deg2rad(tsfloat_get(&yaw_speed_max))); /* the only applied mode in manual operation */
   man_mode_t man_mode = channel_to_man_mode(sw_r);
   if (man_mode == MAN_NOVICE && !gps_valid)
   {
      /* lost gps fix: switch to attitude control */
      man_mode = MAN_RELAXED;
   }
   handle_mode_update(man_mode);
   
   switch (man_mode)
   {
      case MAN_SPORT:
      {
         set_pitch_roll_rates(pitch, roll);
         cm_u_set_acc(f_max / mass * (gas_stick - 0.5));
         break;
      }

      case MAN_RELAXED:
      {
         set_att_angles(pitch, roll);
         set_vertical_spd_or_pos(gas_stick - 0.5, u_baro_pos, u_ultra_pos);
         break;
      }

      case MAN_NOVICE:
      {
         set_horizontal_spd_or_pos(pitch, roll, yaw, ne_gps_pos, u_ultra_pos);
         set_vertical_spd_or_pos(gas_stick - 0.5, u_baro_pos, u_ultra_pos);
         break;
      }
   }
   *hard_off = !((sensor_status & RC_VALID) && sw_l < 0.5);

   return gas_stick > 0.1;
}

