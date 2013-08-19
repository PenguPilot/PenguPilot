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

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "cal_ahrs.h"
#include "ahrs.h"


static ahrs_t ahrs;
static ahrs_t imu;


void cal_ahrs_init(float beta_start, float beta_step, float beta_end)
{
   ahrs_init(&ahrs, AHRS_ACC_MAG, beta_start, beta_step, beta_end);
   ahrs_init(&imu, AHRS_ACC, beta_start, beta_step, beta_end);
}


int cal_ahrs_update(euler_t *euler, marg_data_t *marg_data, float dt)
{
   ahrs_update(&imu, marg_data, dt);
   int status = ahrs_update(&ahrs, marg_data, dt);
   if (status != -1)
   {
      
   }
   return status;
}

