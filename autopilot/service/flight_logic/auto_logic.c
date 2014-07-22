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
  
 Auto Flight Logic Implementation

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
#include <pthread.h>

#include <util.h>
#include <threadsafe_types.h>
#include <opcd_interface.h>

#include "auto_logic.h"
#include "sticks.h"
#include "../main_loop/control_mode.h"
#include "../platform/platform.h"
#include "../util/logger/logger.h"


/* n/e direction in meters: */
static tsfloat_t setp_n;
static tsfloat_t setp_e;

/* up direction in meters: */
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static float setp_u = -1.0;
static bool mode_is_ground = true;
static tsint_t motors_enabled;

/* yaw setpoint in rad: */
static tsfloat_t setp_yaw;

static float hyst_gps_override = 0.0f;
static bool kill_switch = false;
static bool prev_kill_switch = false;


void auto_logic_init(void)
{
   tsfloat_init(&setp_n, 0.0f);
   tsfloat_init(&setp_e, 0.0f);
   tsfloat_init(&setp_yaw, 0.0f);
   tsint_init(&motors_enabled, 0);
}


static void set_att_angles(float pitch, float roll)
{
   vec2_t pr_sp;
   vec2_set(&pr_sp, sticks_pitch_roll_angle_func(pitch), sticks_pitch_roll_angle_func(roll));
   cm_att_set_angles(&pr_sp);
}


bool auto_logic_run(bool *hard_off, bool is_full_auto, uint16_t sensor_status, bool flying, float channels[PP_MAX_CHANNELS], float yaw, vec2_t *ne_gps_pos, float u_baro_pos, float u_ultra_pos, float dt)
{ 
   /* set u position: */
   pthread_mutex_lock(&mutex);
   if (mode_is_ground)
      cm_u_set_ultra_pos(setp_u);
   else
      cm_u_set_baro_pos(setp_u);
   pthread_mutex_unlock(&mutex);

   /* set gps position: */
   vec2_t ne_gps_setpoint;
   vec2_set(&ne_gps_setpoint, tsfloat_get(&setp_n), tsfloat_get(&setp_e));
   cm_att_set_gps_pos(&ne_gps_setpoint);
 
   /* set yaw position: */
   cm_yaw_set_pos(tsfloat_get(&setp_yaw));

   /* safe_auto code: */
   if (!is_full_auto)
   {
      if (sensor_status & RC_VALID)
      {
         float sw_l = channels[CH_SWITCH_L];
         float gas_stick = channels[CH_GAS];
         float pitch = channels[CH_PITCH];
         float roll = channels[CH_ROLL];

         if (sw_l > 0.5)
            kill_switch = true;
         
         if (kill_switch)
         {
            if (!prev_kill_switch)
               LOG(LL_DEBUG, "killed");
            prev_kill_switch = true;
            *hard_off = true;   
         }

         cm_u_a_max_set(sticks_gas_acc_func(gas_stick));
         
         if (!sticks_pitch_roll_in_deadzone(pitch, roll))
            hyst_gps_override = 3.0f;
         hyst_gps_override -= dt;

         if (hyst_gps_override > 0.0)
            set_att_angles(pitch, roll);
      }
      else
      {
         kill_switch = true;
         tsint_init(&motors_enabled, 0);
      }
   }

   return tsint_get(&motors_enabled);
}



int auto_logic_set_n(float val)
{
   LOG(LL_DEBUG, "set_n: %f", val);
   tsfloat_set(&setp_n, val);
   return 0;
}


int auto_logic_set_e(float val)
{
   LOG(LL_DEBUG, "set_e: %f", val);
   tsfloat_set(&setp_e, val);
   return 0;
}


int auto_logic_set_u_ground(float val)
{
   LOG(LL_DEBUG, "set_u_ground: %f", val);
   if (val > 4.0)
      return -1;   
   pthread_mutex_lock(&mutex);
   setp_u = val;
   mode_is_ground = true;
   pthread_mutex_unlock(&mutex);
   return 0;
}


int auto_logic_set_u_msl(float val)
{
   LOG(LL_DEBUG, "set_u_msl: %f", val);
   pthread_mutex_lock(&mutex);
   setp_u = val;
   mode_is_ground = false;
   pthread_mutex_unlock(&mutex);
   return 0;
}


int auto_logic_set_yaw(float val)
{
   tsfloat_set(&setp_yaw, val);
   return 0;
}


int auto_logic_enable_motors(bool enable)
{
   LOG(LL_DEBUG, "enable_motors: %d", enable);
   tsint_set(&motors_enabled, enable); 
   return 0;
}

