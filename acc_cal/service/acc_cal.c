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
  
 ACC Calibration Implementation

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <physics.h>
#include <opcd_interface.h>
#include <threadsafe_types.h>
#include <util.h>

#include "acc_cal.h"


static tsfloat_t acc_bias[3];
static tsfloat_t acc_scale[3];


void acc_cal_init(void)
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
      OPCD_PARAMS_END
   };
   opcd_params_apply(".", params);
}


void acc_cal_apply(vec3_t *acc)
{
   FOR_N(i, 3)
      acc->ve[i] = G_CONSTANT * (acc->ve[i] - tsfloat_get(&acc_bias[i])) / tsfloat_get(&acc_scale[i]);
}

