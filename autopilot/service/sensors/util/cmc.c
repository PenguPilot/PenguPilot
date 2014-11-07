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
  
 Current Magnetometer Compensation (CMC) Implementation

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

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

#include "cmc.h"


static tsfloat_t scale[3];
static tsfloat_t bias;


void cmc_init(void)
{
   ASSERT_ONCE();

   opcd_param_t params[] =
   {
      {"scale_x", &scale[0]},
      {"scale_y", &scale[1]},
      {"scale_z", &scale[2]},
      {"bias", &bias},
      OPCD_PARAMS_END
   };
   opcd_params_apply("cmc.", params);
}


void cmc_apply(vec3_t *mag, const float current)
{
   FOR_N(i, 3)
      mag->ve[i] -= (current - tsfloat_get(&bias)) * tsfloat_get(&scale[i]);
}

