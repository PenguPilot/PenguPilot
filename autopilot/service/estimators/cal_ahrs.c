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
#include "angle_kalman.h"


static ahrs_t ahrs;

static angle_kalman_t pitch_kalman;


void cal_ahrs_init(void)
{
   ASSERT_ONCE();
   tsfloat_t beta;
   tsfloat_t beta_start;
   tsfloat_t beta_step;

   /* read configuration: */
   opcd_param_t params[] =
   {
      {"beta", &beta},
      {"beta_start", &beta_start},
      {"beta_step", &beta_step},
      OPCD_PARAMS_END
   };
   opcd_params_apply("ahrs.", params);
   
   /* initialize AHRS filter: */
   ahrs_init(&ahrs, AHRS_ACC_MAG, tsfloat_get(&beta_start), tsfloat_get(&beta_step), 1.0);
   angle_kalman_init(&pitch_kalman);
}


int cal_ahrs_update(euler_t *euler, const marg_data_t *marg_data,
                    const float mag_decl, const float dt)
{
   int status = ahrs_update(&ahrs, marg_data, dt);
   euler_t _euler;
   /* read euler angles from quaternions: */
   quat_to_euler(&_euler, &ahrs.quat);
   /* apply calibration: */
   euler->yaw = _euler.yaw + mag_decl;
   euler->pitch = _euler.pitch;
   euler->roll = _euler.roll;
   euler_normalize(euler);

   /*float pitch_rate = marg_data->gyro.y;
   float pitch_angle = atan2(marg_data->acc.x, sqrt(marg_data->acc.y * marg_data->acc.y + marg_data->acc.z * marg_data->acc.z));
   printf("%f %f\n", euler->pitch, angle_kalman_run(&pitch_kalman, pitch_rate, pitch_angle, dt));
   */
   return status;
}

