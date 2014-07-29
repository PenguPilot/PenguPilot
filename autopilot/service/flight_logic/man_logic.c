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
#include <float.h>

#include <threadsafe_types.h>
#include <opcd_interface.h>
#include <util.h>
#include <interval.h>

#include "sticks.h"
#include "man_logic.h"
#include "../sensors/util/calibration.h"
#include "../util/logger/logger.h"
#include "../util/math/conv.h"
#include "../util/math/linfunc.h"
#include "../platform/platform.h"
#include "../control/position/navi.h"
#include "../control/position/u_ctrl.h"
#include "../main_loop/control_mode.h"


#define RC_INVAL_MAX_COUNT 300


typedef enum
{
   MAN_SPORT,
   MAN_RELAXED,
   MAN_NOVICE,
   MAN_NONE
}
man_mode_t;

man_mode_t last_mode = MAN_NONE;


static bool always_hard_off = false;
static bool horiz_pos_locked = true;
static int rc_inval_count = 0;
static float yaw_pos_sp = 0.0f;
static float u_pos_sp = -5.0;


static float desired_speed(float err, float p, float max)
{
   float y = p * err;
   if (y > max)
      y = max;
   else if (y < -max)
      y = -max;
   return -y;
}


void man_logic_init(void)
{

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


static void handle_mode_update(man_mode_t mode, float ultra_u_pos)
{
   if (last_mode != mode)
   {
      LOG(LL_INFO, "switching manual mode to: %d", mode);
      u_ctrl_reset();
      if (last_mode != MAN_NONE)
         u_pos_sp = ultra_u_pos;
      last_mode = mode;   
   }
}


static void set_vertical_spd_or_pos(float gas_stick, float ultra_u_pos, float dt)
{
   float inc = sticks_gas_speed_deadzone(gas_stick) * dt;
   if (   !(u_pos_sp < -1.0f && inc < 0.0f)
       && !(u_pos_sp > 3.5f && inc > 0.0f))
   {
      u_pos_sp += inc;
   }
   cm_u_set_ultra_pos(u_pos_sp);
   EVERY_N_TIMES(100, LOG(LL_INFO, "u_pos_sp: %f, u_pos: %f", u_pos_sp, ultra_u_pos));
}


static void set_vertical_spd_or_pos_baro(float gas_stick, float baro_u_pos, float dt)
{
   float inc = sticks_gas_speed_deadzone(gas_stick) * dt;
   if (   !(u_pos_sp < -5.0f && inc < 0.0f)
       && !(u_pos_sp > 100.0f && inc > 0.0f))
   {
      u_pos_sp += inc;
   }
   float err = baro_u_pos - u_pos_sp;
   float spd = desired_speed(err, 1.0, 3.0);
   cm_u_set_spd(spd);
   EVERY_N_TIMES(100, LOG(LL_INFO, "u_pos_sp: %f, u_pos: %f", u_pos_sp, baro_u_pos));
}


static void set_pitch_roll_rates(float pitch, float roll)
{
   vec2_t pr_sp;
   vec2_set(&pr_sp, sticks_pitch_roll_speed_func(pitch), sticks_pitch_roll_speed_func(roll));
   cm_att_set_rates(&pr_sp);
}


static void set_horizontal_spd_or_pos(float pitch, float roll, float yaw, vec2_t *ne_gps_pos, float ultra_u_pos)
{
   if (!sticks_pitch_roll_in_deadzone(pitch, roll) || ultra_u_pos < 0.4)
   {
      /* set GPS speed based on sticks input: */
      vec2_t pitch_roll_spd_sp;
      vec2_set(&pitch_roll_spd_sp, sticks_pitch_roll_gps_speed_func(pitch), sticks_pitch_roll_gps_speed_func(roll));
      vec2_t ne_spd_sp;
      vec2_init(&ne_spd_sp); 
      vec2_rotate(&ne_spd_sp, &pitch_roll_spd_sp, yaw);
      cm_att_set_gps_spd(&ne_spd_sp);
      horiz_pos_locked = false;
   }
   else if (!horiz_pos_locked)
   {
      /* lock GPS position until next sticks activity: */
      navi_reset();
      horiz_pos_locked = true;
      cm_att_set_gps_pos(ne_gps_pos);   
   }
}


static void set_att_angles(float pitch, float roll)
{
   vec2_t pr_sp;
   vec2_set(&pr_sp, sticks_pitch_roll_angle_func(pitch), sticks_pitch_roll_angle_func(roll));
   cm_att_set_angles(&pr_sp);
}



typedef enum
{
   RTL_INIT,
   RTL_ASCEND,
   RTL_RETURN,
   RTL_DESCEND,
   RTL_DONE
} rtl_state_t;

static char *names[5] = {"INIT", "ASCEND", "RETURN", "DESCEND", "DONE"};

static rtl_state_t rtl_state = RTL_INIT;
static rtl_state_t rtl_state_prev = RTL_DONE;


static vec2_t rtl_horiz_sp;
static vec2_t step;



static void tercom_u(float baro_u_pos, float elev)
{
   float spd_max = 2.0f;
   float spd_p = 1.0;
   float safety_delta = 5.0f;
   float safe_elev = safety_delta + elev;
   float err = baro_u_pos - safe_elev;
   float spd = desired_speed(err, spd_p, spd_max);
   cm_u_set_spd(spd);
   EVERY_N_TIMES(100, LOG(LL_INFO, "u_pos: %f, safe_elev: %f", baro_u_pos, safe_elev));
}


static bool rtl_tercom(bool flying, bool gps_valid, vec2_t *ne_gps_pos, float baro_u_pos, float ultra_u_pos, float elev, float dt)
{
   bool motors_off = false;
   float spd_max = 2.0f;
   float spd_p = 1.0;
   float safety_delta = 6.0f;
   float safe_elev = safety_delta + elev;

   /* keep yaw speed low: */
   cm_yaw_set_spd(0.0f);
   
   switch (rtl_state)
   {
      case RTL_INIT:
      {
         if (flying)
         {
            vec2_set(&rtl_horiz_sp, ne_gps_pos->x, ne_gps_pos->y);
            /* lock position before ascending: */
            cm_att_set_gps_pos(&rtl_horiz_sp);
            
            vec2_t origin_dir;
            vec2_init(&origin_dir);
            vec2_normalize(&origin_dir, &rtl_horiz_sp);
            vec2_init(&step);
            vec2_scale(&step, &origin_dir, -spd_max * dt);

            float err = baro_u_pos - safe_elev;
            float spd = desired_speed(err, spd_p, spd_max);
            cm_u_set_spd(spd);

            rtl_state = RTL_ASCEND;
         }
         else
         {
            rtl_state = RTL_DONE;   
         }
         break;
      }

      case RTL_ASCEND:
         if (fabs(baro_u_pos - safe_elev) > 2.0)
         {
            /* increase altitude */   
            cm_u_set_spd(spd_max);
         }
         else
         {
            rtl_state = RTL_RETURN;   
         }
         break;

      case RTL_RETURN:
         if (sqrt(ne_gps_pos->x * ne_gps_pos->x + ne_gps_pos->y * ne_gps_pos->y) > 5.0)
         {
            /* update N,E position setpoint towards starting point */
            vec2_add(&rtl_horiz_sp, &rtl_horiz_sp, &step);
            cm_att_set_gps_pos(&rtl_horiz_sp);
            /* follow elevation map data (U) */
            float err = baro_u_pos - safe_elev;
            float spd = desired_speed(err, spd_p, spd_max);
            cm_u_set_spd(spd);
         }
         else
         {
            rtl_state = RTL_DESCEND;   
         }
         break;

      case RTL_DESCEND:
         if (ultra_u_pos > 0.4f)
         {
            /* decrease altitude */   
            if (ultra_u_pos > 4.0f)
               cm_u_set_spd(-spd_max); /* descent speed above 4 meters */
            else if (ultra_u_pos > 2.0f)
               cm_u_set_spd(-spd_max / 2); /* descent speed above 2 meters */
            else
               cm_u_set_spd(-spd_max / 4); /* final landing speed */
         }
         else
         {
            rtl_state = RTL_DONE;
         }
         break;

      case RTL_DONE:
         //motors_off = true;
         break;
   }
   
   if (rtl_state != rtl_state_prev)
   {
      LOG(LL_DEBUG, "new state: %s", names[rtl_state]);
   }
   rtl_state_prev = rtl_state;

   return motors_off;
}


bool man_logic_run(bool *hard_off, uint16_t sensor_status, bool flying, float channels[PP_MAX_CHANNELS],
                  float yaw, vec2_t *ne_gps_pos, float baro_u_pos, float ultra_u_pos, float f_max, float mass, float dt, float elev)
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
      
      rc_inval_count++;
      if (rc_inval_count >= RC_INVAL_MAX_COUNT)
      {
         /* too much; bring it down */
         if (rc_inval_count == RC_INVAL_MAX_COUNT)
            LOG(LL_ERROR, "performing emergency landing");
        
         if (rtl_tercom(flying, sensor_status & GPS_VALID, ne_gps_pos, baro_u_pos, ultra_u_pos, elev, dt))
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
   vec2_rotate(&pr, &pr, sticks_rotation());
   float pitch = pr.x;
   float roll = pr.y;

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
   handle_mode_update(man_mode, ultra_u_pos);

   if (ultra_u_pos < 0.5 || !flying)
   {
      /* reset yaw setpoint if we are too low: */
      yaw_pos_sp = yaw;
      cm_yaw_set_spd(sticks_yaw_speed_deadzone(yaw_stick));
   }
   else
   {
      /* if the mode request it and we are flying high enough,
       * allow fixed yaw control: */
      yaw_pos_sp += sticks_yaw_speed_deadzone(yaw_stick) * dt;
      yaw_pos_sp = norm_angle_0_2pi(yaw_pos_sp);
      cm_yaw_set_pos(yaw_pos_sp);
   }

   switch (man_mode)
   {
      case MAN_SPORT:
      {
         set_pitch_roll_rates(pitch, roll);
         cm_u_set_acc(sticks_gas_acc_func(gas_stick));
         break;
      }

      case MAN_RELAXED:
      {
         set_att_angles(pitch, roll);
         //cm_u_set_acc(sticks_gas_acc_func(gas_stick));
         tercom_u(baro_u_pos, elev);
         cm_u_a_max_set(sticks_gas_acc_func(gas_stick));
         break;
      }

      case MAN_NOVICE:
      {
         set_horizontal_spd_or_pos(pitch, roll, yaw, ne_gps_pos, ultra_u_pos);
         set_vertical_spd_or_pos(gas_stick, ultra_u_pos, dt);
         //cm_u_set_spd(gas_stick);
         break;
      }

      default:
         break;
   }

   if (((sensor_status & RC_VALID) && sw_l > 0.5))
      *hard_off = true;

   //cm_u_a_max_set(FLT_MAX); /* this limit applies only in auto mode */
   return gas_stick > -0.8;
}

