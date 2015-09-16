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
  
 Stabilizing PIID Controller Implementation

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau
 Copyright (C) 2013 Alexander Barth, Control Engineering Group, TU Ilmenau
 Copyright (C) 2013 Benjamin Jahn, Control Engineering Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <util.h>
#include <threadsafe_types.h>
#include <opcd_interface.h>
#include <logger.h>
#include <filter.h>
#include <pid.h>
#include "piid.h"


/* configuration parameters: */
static tsfloat_t att_kp;
static tsfloat_t att_ki;
static tsfloat_t att_kd;
static tsfloat_t yaw_kp;
static tsfloat_t yaw_ki;
static tsfloat_t yaw_kd;
static tsfloat_t filt_d;


static pid_controller_t ctrl[3];


void piid_init(float dt)
{
   ASSERT_ONCE();

   opcd_param_t params[] =
   {
      {"att_kp", &att_kp},
      {"att_ki", &att_ki},
      {"att_kd", &att_kd},
      {"yaw_kp", &yaw_kp},
      {"yaw_ki", &yaw_ki},
      {"yaw_kd", &yaw_kd},
      {"filt_d", &filt_d},
      OPCD_PARAMS_END
   };
   opcd_params_apply(".", params);
   
   
   pid_init(&ctrl[0], &att_kp, &att_ki, &att_kd, &filt_d, NULL);
   pid_init(&ctrl[1], &att_kp, &att_ki, &att_kd, &filt_d, NULL);
   pid_init(&ctrl[2], &yaw_kp, &yaw_ki, &yaw_kd, &filt_d, NULL);
}


void piid_int_enable(int val)
{
   FOR_N(i, 3)
      ctrl[i].i_enabled = val;
}


void piid_reset(void)
{
   FOR_N(i, 3)
      tsfloat_set(&ctrl[i].sum_error, 0.0);
}


void piid_run(float u_ctrl[3], float gyro[3], float rc[3], float dt)
{
   FOR_N(i, 3)
      u_ctrl[i] = pid_control(&ctrl[i], rc[i] - gyro[i], 0.0, dt);
   
   //workaround allowing quadcopter to be stable on the ground
   double gyro_noise_value = 0.0005; //must be set very carefully  
      
   if ((abs(rc[0]-gyro[0]) < gyro_noise_value) && (abs(rc[1]-gyro[1]) < gyro_noise_value))
    {
     tsfloat_set(&ctrl[0].sum_error, 0.0);
     tsfloat_set(&ctrl[1].sum_error, 0.0);
    }
}

