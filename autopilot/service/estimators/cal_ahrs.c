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
#include "ahrs.h"
#include "../util/math/conv.h"
#include "../util/math/adams5.h"


static tsfloat_t beta_end;
static tsfloat_t beta_start;
static tsfloat_t beta_step;
static ahrs_t ahrs;


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
   ahrs_init(&ahrs, AHRS_ACC_MAG, tsfloat_get(&beta_start), tsfloat_get(&beta_step), tsfloat_get(&beta_end));
}


int cal_ahrs_update(euler_t *euler, const marg_data_t *marg_data,
                    const float mag_decl, const float dt)
{
   int ret = ahrs_update(&ahrs, marg_data, dt);
   euler_t _euler;
   quat_to_euler(&_euler, &ahrs.quat);

   euler->pitch = _euler.pitch;
   euler->roll = _euler.roll;
   euler->yaw = _euler.yaw + deg2rad(mag_decl);
   euler_normalize(euler);
   return ret;
}

