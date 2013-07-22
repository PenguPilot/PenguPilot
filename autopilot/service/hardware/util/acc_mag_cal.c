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
  
 ACC/MAG Calibration

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <opcd_interface.h>
#include <threadsafe_types.h>
#include <util.h>

#include "acc_mag_cal.h"


/* calibration data: */
static tsfloat_t acc_bias[3];
static tsfloat_t acc_scale[3];
static tsfloat_t mag_bias[3];
static tsfloat_t mag_scale[3];


void acc_mag_cal_init(void)
{
   ASSERT_ONCE();

   /* load calibration: */
   opcd_param_t params[] =
   {
      /* acc bias: */
      {"acc_bias_x", &acc_bias[0]},
      {"acc_bias_y", &acc_bias[1]},
      {"acc_bias_z", &acc_bias[2]},
      /* acc scale: */
      {"acc_scale_x", &acc_scale[0]},
      {"acc_scale_y", &acc_scale[1]},
      {"acc_scale_z", &acc_scale[2]},
      /* mag bias: */
      {"mag_bias_x", &mag_bias[0]},
      {"mag_bias_y", &mag_bias[1]},
      {"mag_bias_z", &mag_bias[2]},
      /* mag scale: */
      {"mag_scale_x", &mag_scale[0]},
      {"mag_scale_y", &mag_scale[1]},
      {"mag_scale_z", &mag_scale[2]},
      OPCD_PARAMS_END
   };
   opcd_params_apply("cal.", params);
}


void acc_mag_cal_apply(vec3_t *acc, vec3_t *mag)
{
   FOR_N(i, 3)
   {
      acc->vec[i] = (acc->vec[i] - tsfloat_get(&acc_bias[i]) / tsfloat_get(&acc_scale[i]));
      mag->vec[i] = (mag->vec[i] - tsfloat_get(&mag_bias[i]) / tsfloat_get(&mag_scale[i]));
   }
}

