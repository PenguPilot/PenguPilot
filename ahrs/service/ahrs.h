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
  
 Madgwick AHRS Algorithm Interface
 See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau
 Copyright (C) 2014 SOH Madgwick, X-IO Technologies

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __AHRS_H__
#define __AHRS_H__


#include <marg_data.h>
#include <math/quat.h>
#include <math/adams5.h>


typedef enum
{
   AHRS_ACC_MAG,
   AHRS_ACC
}
ahrs_type_t;


typedef struct
{
   real_t beta; /* 2 * beta (Kp) */
   real_t beta_step;
   real_t beta_end;
   ahrs_type_t type; /* AHRS_ACC_MAG or AHRS_ACC */
   quat_t quat; /* quaternion of sensor frame relative to auxiliary frame */
   adams5_t adams5;
}
ahrs_t;


void ahrs_init(ahrs_t *ahrs, ahrs_type_t type, real_t beta_start, real_t beta_step, real_t beta_end);

/*
 * returns -1 if the ahrs is not ready
 *          1 if the ahrs became ready
 *          0 on normal operation
 */
int ahrs_update(ahrs_t *ahrs, const marg_data_t *marg_data, const real_t dt);


#endif /* __AHRS_H__ */

