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
#include <interval.h>

#include "man_logic.h"
#include "../hardware/util/calibration.h"
#include "../util/logger/logger.h"
#include "../util/math/conv.h"
#include "../util/math/linfunc.h"
#include "../hardware/platform/platform.h"
#include "../control/position/navi.h"
#include "../control/position/u_ctrl.h"
#include "../main_loop/control_mode.h"


#define RC_INVAL_MAX_COUNT 300


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
static tsfloat_t gas_acc_max;
static tsfloat_t sticks_rotation;

static bool always_hard_off = false;
static bool vert_pos_locked = false;
static bool horiz_pos_locked = true;
static int rc_inval_count = 0;
static float yaw_pos_sp = 0.0f;
static float pos = -5.0;


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
      {"gas_acc_max", &gas_acc_max},
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



static void set_vertical_spd_or_pos(float gas_stick, float baro_u_pos, float ultra_u_pos)
{
   float dz = tsfloat_get(&gas_deadzone);
   float vmax = tsfloat_get(&vert_speed_max);
   float inc = stick_dz(gas_stick, dz) * vmax * 0.00333333;
   if (!(pos - baro_u_pos < -5.0f && inc < 0.0f))
      pos += stick_dz(gas_stick, dz) * vmax * 0.00333333;
   cm_u_set_baro_pos(pos);
   EVERY_N_TIMES(100, LOG(LL_INFO, "u_pos_sp: %f, u_pos: %f", pos, baro_u_pos));
}


static void set_pitch_roll_rates(float pitch, float roll)
{
   float dz = tsfloat_get(&gps_deadzone);
   float rate_max = deg2rad(tsfloat_get(&pitch_roll_speed_max));
   vec2_t pr_sp;
   vec2_set(&pr_sp, rate_max * pitch, rate_max * roll);
   cm_att_set_rates(&pr_sp);
}


static void set_horizontal_spd_or_pos(float pitch, float roll, float yaw, vec2_t *ne_gps_pos, float ultra_u_pos)
{
   if (1) //sqrt(pitch * pitch + roll * roll) > tsfloat_get(&gps_deadzone) || ultra_u_pos < 0.4)
   {
      /* set GPS speed based on sticks input: */
      float vmax_sqrt = sqrt(tsfloat_get(&horiz_speed_max));
      float dz = tsfloat_get(&gps_deadzone);
      vec2_t pitch_roll_spd_sp;
      vec2_set(&pitch_roll_spd_sp, vmax_sqrt * stick_dz(pitch, dz), vmax_sqrt * stick_dz(roll, dz));
      vec2_t ne_spd_sp;
      vec2_init(&ne_spd_sp); 
      vec2_rotate(&ne_spd_sp, &pitch_roll_spd_sp, yaw);
      cm_att_set_gps_spd(&ne_spd_sp);
      horiz_pos_locked = false;
   }
   else if (!horiz_pos_locked)
   {
      LOG(LL_INFO, "horizontal position lock at relative N: %fm, E: %fm", ne_gps_pos->x, ne_gps_pos->y);
      /* lock GPS position until next sticks activity: */
      navi_reset();
      horiz_pos_locked = true;
      cm_att_set_gps_pos(ne_gps_pos);   
   }
}


void set_att_angles(float pitch, float roll)
{
   float dz = tsfloat_get(&gps_deadzone);
   float angle_max = deg2rad(tsfloat_get(&pitch_roll_angle_max));
   vec2_t pr_sp;
   vec2_set(&pr_sp, angle_max * pitch, angle_max * roll);
   cm_att_set_angles(&pr_sp);
}


static bool emergency_landing(bool gps_valid, vec2_t *ne_gps_pos, float ultra_u_pos)
{
   vert_pos_locked = false;
   cm_u_set_spd(-0.2);
   
   if (gps_valid)
      set_horizontal_spd_or_pos(0.0f, 0.0f, 0.0f, ne_gps_pos, ultra_u_pos);
   else
      set_att_angles(0.0f, 0.0f);
   
   cm_yaw_set_spd(0.0f);

   if (ultra_u_pos < 0.4f)
      return true;

   return false;
}


bool man_logic_run(bool *hard_off, uint16_t sensor_status, bool flying, float channels[MAX_CHANNELS], float yaw, vec2_t *ne_gps_pos, float baro_u_pos, float ultra_u_pos, float f_max, float mass, float dt)
{
   if (always_hard_off)
   {
      *hard_off = true;
      return false;
   }

   if (!(sensor_status & RC_VALID))
   {
      if (rc_inval_count == 0)
         LOG(LL_ERROR, "rc signal invalid!");
      /* restore previous channels: */
      memset(channels, 0, sizeof(float) * MAX_CHANNELS);
      channels[CH_GAS] = 0.3;
      rc_inval_count++;
      if (rc_inval_count >= RC_INVAL_MAX_COUNT)
      {
         /* too much; bring it down */
         if (rc_inval_count == RC_INVAL_MAX_COUNT)
            LOG(LL_ERROR, "performing emergency landing");
         
         if (emergency_landing(sensor_status & GPS_VALID, ne_gps_pos, ultra_u_pos))
         {
            if (!always_hard_off)
               LOG(LL_ERROR, "emergency landing complete; disabling actuators");
            always_hard_off = true;
            return false;
         }
      }
   }
   else
      rc_inval_count = 0;   
   
   vec2_t pr;
   vec2_set(&pr, channels[CH_PITCH], channels[CH_ROLL]);
   vec2_rotate(&pr, &pr, deg2rad(tsfloat_get(&sticks_rotation)));
   float pitch = pr.ve[0];
   float roll = pr.ve[1];

   float yaw_stick = channels[CH_YAW];
   float gas_stick = channels[CH_GAS];
   float sw_l = channels[CH_SWITCH_L];
   float sw_r = channels[CH_SWITCH_R];
   bool gps_valid = (sensor_status & GPS_VALID) ? true : false;

   man_mode_t man_mode = channel_to_man_mode(sw_r);
   if (man_mode == MAN_NOVICE && !gps_valid)
   {
      /* lost gps fix: switch to attitude control */
      man_mode = MAN_RELAXED;
   }
   handle_mode_update(man_mode);
   

   if (ultra_u_pos < 0.3)
   {
      /* reset yaw setpoint if we are too low: */
      yaw_pos_sp = yaw;
      /* control only yaw speed */
      cm_yaw_set_spd(stick_dz(yaw_stick, 0.075) * deg2rad(tsfloat_get(&yaw_speed_max)));
   }
   else
   {
      /* if the mode request it and we are flying high enough,
       * allow fixed yaw control: */
      yaw_pos_sp += stick_dz(yaw_stick, 0.075) * deg2rad(tsfloat_get(&yaw_speed_max)) * dt;
      yaw_pos_sp = norm_angle_0_2pi(yaw_pos_sp);
      cm_yaw_set_pos(yaw_pos_sp);
   }

   switch (man_mode)
   {
      case MAN_SPORT:
      {
         set_pitch_roll_rates(pitch, roll);
         cm_u_set_acc(tsfloat_get(&gas_acc_max) * (gas_stick - 0.5));
         break;
      }

      case MAN_RELAXED:
      {
         set_att_angles(pitch, roll);
         cm_u_set_acc(tsfloat_get(&gas_acc_max) * (gas_stick - 0.5));
         //set_vertical_spd_or_pos(gas_stick - 0.5, baro_u_pos, ultra_u_pos);
         break;
      }

      case MAN_NOVICE:
      {
         set_horizontal_spd_or_pos(pitch, roll, yaw, ne_gps_pos, ultra_u_pos);
         set_vertical_spd_or_pos(gas_stick - 0.5, baro_u_pos, ultra_u_pos);
         break;
      }
   }

   if (((sensor_status & RC_VALID) && sw_l > 0.5))
      *hard_off = true;

   return gas_stick > 0.1;
}

