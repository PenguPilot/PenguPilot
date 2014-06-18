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


#include <math.h>

#include <util.h>
#include <string.h>
#include <opcd_interface.h>
#include <threadsafe_types.h>

#include "cal_ahrs.h"
#include "../util/math/conv.h"
#include "../util/math/adams5.h"


static float pry[3] = {0.0, 0.0, 0.0};
static tsfloat_t beta_end;
static tsfloat_t beta_start;
static tsfloat_t beta_step;
static adams5_t adams;
static float beta;


void cal_ahrs_init(void)
{
   ASSERT_ONCE();

   /* read configuration: */
   opcd_param_t params[] =
   {
      {"beta", &beta_end},
      {"beta_start", &beta_start},
      {"beta_step", &beta_step},
      OPCD_PARAMS_END
   };
   opcd_params_apply("ahrs.", params);
   beta = tsfloat_get(&beta_start);
   adams5_init(&adams, 3);
}


int cal_ahrs_update(euler_t *euler, const marg_data_t *marg_data,
                    const float mag_decl, const float dt)
{
   int ret = 0;
   beta -= tsfloat_get(&beta_step);
   if (beta < tsfloat_get(&beta_end))
   {
      beta = tsfloat_get(&beta_end);
      ret = 1;
   }
   float in[3] = {marg_data->gyro.y, -marg_data->gyro.x, marg_data->gyro.z};
   /* run integrator: */
   adams5_run(&adams, pry, in, dt, 1);
   
   float pitch_ref = atan2(marg_data->acc.x, -marg_data->acc.z);
   pry[0] = pry[0] * (1.0 - beta) + pitch_ref * beta;
   float roll_ref = atan2(marg_data->acc.y, -marg_data->acc.z);
   pry[1] = pry[1] * (1.0 - beta) + roll_ref * beta;
   float mag_g_x = marg_data->mag.x * cos(euler->pitch) + marg_data->mag.y * sin(euler->pitch)*sin(euler->roll) + marg_data->mag.z * sin(euler->pitch)*cos(euler->roll);
   float mag_g_y = marg_data->mag.y * cos(euler->roll) - marg_data->mag.z * sin(euler->roll);
   
   /* yaw, mostly stolen from multiwii: */
   if (pry[2] > M_PI) pry[2] = -M_PI * 2.0 + pry[2];
   if (pry[2] < -M_PI) pry[2] = M_PI * 2.0 - pry[2];
   float yaw_ref = atan2(-mag_g_y, mag_g_x);
   float hdiff = yaw_ref - pry[2];
   if (hdiff > M_PI) hdiff = hdiff - 2.0 * M_PI;  // choose CCW because more nearby than CW
   if (hdiff < -M_PI) hdiff = 2.0 * M_PI + hdiff; // choose CW because more nearby than CCW
   pry[2] = pry[2] + hdiff * beta;  // the correction of the gyro yaw
   
   /* output: */
   euler->pitch = pry[0];
   euler->roll = -pry[1];
   euler->yaw = pry[2] + deg2rad(mag_decl);
   euler_normalize(euler);
   return ret;
}

