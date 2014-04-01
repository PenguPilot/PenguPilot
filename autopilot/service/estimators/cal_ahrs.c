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


void cal_ahrs_init(float beta_start, float beta_step)
{
   ASSERT_ONCE();
   tsfloat_t beta_end;

   /* read configuration: */
   opcd_param_t params[] =
   {
      {"beta", &beta_end},
      OPCD_PARAMS_END
   };
   opcd_params_apply("ahrs.", params);
   
   /* initialize AHRS and IMU filters: */
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
      euler->yaw = ahrs_euler.yaw + mag_decl;
      euler->pitch = imu_euler.pitch;
      euler->roll = imu_euler.roll;
      euler_normalize(euler);
      EVERY_N_TIMES(200, LOG(LL_INFO, "%f %f %f", rad2deg(euler->pitch),
                                                  rad2deg(euler->roll),
                                                  rad2deg(euler->yaw)));
   }
   else
   {
      memset(euler, 0, sizeof(euler_t));
   }
   return status;
}

