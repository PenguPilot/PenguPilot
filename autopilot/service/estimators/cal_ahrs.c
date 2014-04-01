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
  
 Calibrated AHRS Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <util.h>
#include <string.h>
#include <opcd_interface.h>
#include <threadsafe_types.h>

#include "cal_ahrs.h"
#include "ahrs.h"
#include "../util/logger/logger.h"
#include "../util/math/conv.h"


static ahrs_t ahrs;
static ahrs_t imu;
static tsfloat_t yaw_bias;
static tsfloat_t pitch_bias;
static tsfloat_t roll_bias;


void cal_ahrs_init(float beta_start, float beta_step)
{
   ASSERT_ONCE();
   tsfloat_t beta_end;

   /* read configuration and initialize scl gates: */
   opcd_param_t params[] =
   {
      {"yaw_bias", &yaw_bias},
      {"pitch_bias", &pitch_bias},
      {"roll_bias", &roll_bias},
      {"beta", &beta_end},
      OPCD_PARAMS_END
   };
   opcd_params_apply("ahrs.", params);
   LOG(LL_DEBUG, "yaw_bias: %f, pitch_bias: %f, roll_bias: %f",
       tsfloat_get(&yaw_bias),
       tsfloat_get(&pitch_bias),
       tsfloat_get(&roll_bias));

   ahrs_init(&ahrs, AHRS_ACC_MAG, beta_start, beta_step, tsfloat_get(&beta_end));
   ahrs_init(&imu, AHRS_ACC, beta_start, beta_step, tsfloat_get(&beta_end));
}


int cal_ahrs_update(euler_t *euler, marg_data_t *marg_data, float mag_decl, float dt)
{
   ahrs_update(&imu, marg_data, dt);
   int status = ahrs_update(&ahrs, marg_data, dt);
   if (status != -1)
   {
      euler_t ahrs_euler; /* yaw */
      euler_t imu_euler; /* pitch/roll */
      /* read euler angles from quaternions: */
      quat_to_euler(&ahrs_euler, &ahrs.quat);
      quat_to_euler(&imu_euler, &imu.quat);
      /* apply calibration: */
      euler->yaw = ahrs_euler.yaw + deg2rad(tsfloat_get(&yaw_bias)) + mag_decl;
      euler->pitch = ahrs_euler.pitch + deg2rad(tsfloat_get(&pitch_bias));
      euler->roll = ahrs_euler.roll + deg2rad(tsfloat_get(&roll_bias));
      EVERY_N_TIMES(200, LOG(LL_INFO, "%f %f %f", euler->pitch, euler->roll, euler->yaw));
      euler_normalize(euler);
   }
   else
   {
      memset(euler, 0, sizeof(euler_t));
   }
   return status;
}

