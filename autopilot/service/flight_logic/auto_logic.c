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


#include <pthread.h>
#include <threadsafe_types.h>
#include <opcd_interface.h>
#include <util.h>

#include "auto_logic.h"
#include "../main_loop/control_mode.h"


/* n/e direction in meters: */
static tsfloat_t setp_n;
static tsfloat_t setp_e;

/* up direction in meters: */
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static float setp_u = 1.0;
static bool mode_is_ground = true;

/* yaw setpoint in rad: */
static tsfloat_t setp_yaw;


void auto_logic_init(void)
{
   tsfloat_init(&setp_n, 0.0f);
   tsfloat_init(&setp_e, 0.0f);
   tsfloat_init(&setp_yaw, 0.0f);
}


void auto_logic_run(bool is_full_auto, uint16_t sensor_status, bool flying, float channels[MAX_CHANNELS], float yaw, vec2_t *ne_gps_pos, float u_baro_pos, float u_ultra_pos)
{
   int test_pitch_roll = 0;
   if (test_pitch_roll)
   {
      vec2_t angles = {{0.0f, 0.0f}};
      cm_att_set_angles(angles);
      cm_yaw_set_spd(0.0f);
      cm_u_set_acc(channels[CH_GAS]);
   }
   else
   {
      int u_is_manual = 1;
      pthread_mutex_lock(&mutex);
      if (!u_is_manual)
      {
         if (mode_is_ground)
            cm_u_set_ultra_pos(setp_u);
         else
            cm_u_set_baro_pos(setp_u);
      }
      else
         cm_u_set_acc(channels[CH_GAS]);
      pthread_mutex_unlock(&mutex);
      
      int yaw_is_manual = 1;
      if (yaw_is_manual)
         cm_yaw_set_spd(channels[CH_YAW] * 2.0f);

      vec2_t ne_gps_setpoint = {tsfloat_get(&setp_n), tsfloat_get(&setp_e)};
      cm_att_set_gps_pos(ne_gps_setpoint);
   }
}



int auto_logic_set_n(float val)
{
   tsfloat_set(&setp_n, val);
   return 0;
}


int auto_logic_set_e(float val)
{
   tsfloat_set(&setp_e, val);
   return 0;
}


int auto_logic_set_u_ground(float val)
{
   if (val > 6.0)
      return -1;   
   pthread_mutex_lock(&mutex);
   setp_u = val;
   mode_is_ground = true;
   pthread_mutex_unlock(&mutex);
   return 0;
}


int auto_logic_set_u_msl(float val)
{
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

