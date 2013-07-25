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
  
 Control Modes Inmplementation

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <threadsafe_types.h>
#include <opcd_interface.h>
#include "control_mode.h"
#include "main_loop.h"
#include "../filters/filter.h"
#include "../hardware/util/calibration.h"
#include "../util/math/conv.h"


static tsfloat_t stick_pitch_roll_p;
static tsfloat_t stick_pitch_roll_angle_max;
static tsfloat_t stick_yaw_p;

static calibration_t rc_cal;
static Filter1 rc_valid_filter;


void cm_init(void)
{
   cal_init(&rc_cal, 3, 500);
   filter1_lp_init(&rc_valid_filter, 0.5, REALTIME_PERIOD, 1);

   /* read parameters: */
   opcd_param_t params[] =
   {
      {"pitch_roll_p", &stick_pitch_roll_p},
      {"pitch_roll_angle_max", &stick_pitch_roll_angle_max},
      {"yaw_p", &stick_yaw_p},
      OPCD_PARAMS_END
   };
   opcd_params_apply("sticks.", params);
}


void cm_update(control_mode_t *cm, uint16_t sensor_status, float channels[MAX_CHANNELS])
{
   float rc_valid_f = (sensor_status & RC_VALID) ? 1.0f : 0.0f;
   filter1_run(&rc_valid_filter, &rc_valid_f, &rc_valid_f);
   int rc_valid = rc_valid_f > 0.5f;
   float pitch, roll, yaw, gas, sw;
   if (!rc_valid)
   {
      pitch = 0.0f;
      roll = 0.0f;
      yaw = 0.0f;
      gas = 0.0f;
      sw = 0.0f;
   }
   else
   {
      float cal_channels[3] = {channels[CH_PITCH], channels[CH_ROLL], channels[CH_YAW]};
      cal_sample_apply(&rc_cal, cal_channels);
      pitch = cal_channels[0];
      roll = cal_channels[1];
      yaw = cal_channels[2];
      gas = channels[CH_GAS];
      sw = channels[CH_SWITCH];
   }

   /* select mode */
   cm->xy.type = XY_GPS_SPEED;
   cm->xy.global = 1;

   /* fill data accoring to mode */
   if (cm->xy.type == XY_ATT_RATE)
   {
      float p = tsfloat_get(&stick_pitch_roll_p);
      cm->xy.setp.x = p * pitch;
      cm->xy.setp.y = p * roll;
   }
   else if (cm->xy.type == XY_ATT_POS)
   {
      float a = deg2rad(tsfloat_get(&stick_pitch_roll_angle_max));
      cm->xy.setp.x = a * pitch;
      cm->xy.setp.y = a * roll;
   }
   else if (cm->xy.type == XY_GPS_SPEED)
   {
      float p = tsfloat_get(&stick_pitch_roll_p);
      cm->xy.setp.x = p * pitch;   
      cm->xy.setp.y = p * roll;   
   }
   cm->z.type = Z_AUTO; //Z_STICK;
   cm->z.setp = gas;
   cm->yaw.type = YAW_STICK;
   cm->yaw.setp = yaw * tsfloat_get(&stick_yaw_p);
   cm->motors_enabled = rc_valid && sw > 0.5;
}

