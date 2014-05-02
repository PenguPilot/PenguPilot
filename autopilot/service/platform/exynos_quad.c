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
  
 ARCADE Quadrotor Platform

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology
 Copyright (C) 2013 Alexander Barth, Ilmenau University of Technology
 Copyright (C) 2013 Benjamin Jahn, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <stddef.h>
#include <unistd.h>
#include <malloc.h>
#include <math.h>

#include <util.h>
#include "inv_coupling.h"
#include "quad.h"
#include "arcade_quad.h"
#include "platform.h"
#include "../util/logger/logger.h"
#include "../util/math/quat.h"
#include <i2c/i2c.h>

/* hardware includes: */
#include "../hardware/drivers/scl_gps/scl_gps.h"
#include "../hardware/drivers/i2cxl/i2cxl_reader.h"
#include "../hardware/drivers/ms5611/ms5611_reader.h"
#include "../hardware/drivers/arduino_escs/arduino_escs.h"
#include "../hardware/drivers/scl_power/scl_power.h"
#include "../hardware/drivers/scl_rc/scl_rc.h"
#include "../hardware/util/rc_channels.h"
#include "../hardware/util/gps_data.h"
#include "../../arduino/service/ppm_common.h"
#include "freeimu_04.h"


static i2c_bus_t i2c_4;
static rc_channels_t rc_channels;
static uint8_t channel_mapping[MAX_CHANNELS] =  {0, 1, 3, 2, 4, 5}; /* pitch: 0, roll: 1, yaw: 3, gas: 2, switch left: 4, switch right: 5 */
static float channel_scale[MAX_CHANNELS] =  {1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f};
static freeimu_04_t marg;


static int read_rc(float channels[MAX_CHANNELS])
{
   float dsl_channels[SCL_MAX_CHANNELS];
   int ret = scl_rc_read(dsl_channels);
   
   /* for (int i = 0; i < RC_DSL_CHANNELS; i++)
      printf("(%d, %f) ", i, dsl_channels[i]);
      printf("\n");
   */

   for (int c = 0; c < MAX_CHANNELS; c++)
   {
      channels[c] = rc_channels_get(&rc_channels, dsl_channels, c);
   }
   return ret;
}


static int read_marg(marg_data_t *marg_data)
{
   int ret = freeimu_04_read(marg_data, &marg);
   quat_t q;
   quat_init_axis(&q, 0, 1, 0, M_PI);
   quat_rot_vec(&marg_data->acc, &marg_data->acc, &q);
   quat_rot_vec(&marg_data->gyro, &marg_data->gyro, &q);
   quat_rot_vec(&marg_data->mag, &marg_data->mag, &q);
   quat_init_axis(&q, 0, 0, 1, -M_PI / 4.0f);
   quat_rot_vec(&marg_data->acc, &marg_data->acc, &q);
   quat_rot_vec(&marg_data->gyro, &marg_data->gyro, &q);
   quat_rot_vec(&marg_data->mag, &marg_data->mag, &q);
   return ret;
}


static int ultra_dummy_read(float *dist)
{
	*dist = .1f;
	return 0;
}


static float force_to_esc(float force, float volt)
{
   if (force < 0.f)
   {
      return 0.f;
   }

   const float a = 1.2526e-07;
   const float b = -3.3937e-03;
   const float c = -1.3746e+00;
   const float d = 1.3284e-04;
   const float e = 2.0807e+01;
   float pwm = ((sqrtf((b + d * volt) * (b + d * volt) - 4.0f * a * (c * volt + e - force)) - b - d * volt) / (2.0f * a));

   return (pwm - 10000.0f) / 10000.0f;
}


int exynos_quad_init(platform_t *plat, int override_hw)
{
   ASSERT_ONCE();
   THROW_BEGIN();

   float rpm_min = 1340.99f;
   float rpm_max = 7107.6f;
   plat->rpm_square_min = rpm_min * rpm_min;
   plat->rpm_square_max = rpm_max * rpm_max;

   /* arm length */
   const float length = 0.2025f;

   /* F = c * rpm ^ 2 */
   const float c = 1.5866e-007f;

   /* tau = d * rpm ^ 2 */
   const float d = 4.0174e-009f;

   const float imtx1 = 1.0f / (4.0f * c);
   const float imtx2 = 1.0f / (2.0f * c * length);
   const float imtx3 = 1.0f / (4.0f * d);

   plat->imtx1 = imtx1;
   plat->imtx2 = imtx2;
   plat->imtx3 = imtx3;

   /* inverse coupling matrix for ARCADE quadrotor: */
   const float icmatrix[4 * N_MOTORS] =
   {         /* gas     roll    pitch    yaw */
      /* m1 */ imtx1,   0.0f, -imtx2, -imtx3,
      /* m3 */ imtx1, -imtx2,   0.0f,  imtx3,
      /* m4 */ imtx1,  imtx2,   0.0f,  imtx3,
      /* m2 */ imtx1,   0.0f,  imtx2, -imtx3,
   };

   LOG(LL_INFO, "ic matrix [gas roll pitch yaw]");
   FOR_N(i, 4)
   {
      LOG(LL_INFO, "[m%d]: %.0f, %.0f, %.0f, %.0f", i,
          icmatrix[i*4], icmatrix[i*4+1],
          icmatrix[i*4+2], icmatrix[i*4+3]);
   }
   LOG(LL_INFO, "setting platform parameters");
   
  
   plat->max_thrust_n = 40.0f;
   plat->mass_kg = 1.1f;
   plat->n_motors = N_MOTORS;

   LOG(LL_INFO, "initializing inverse coupling matrix");
   inv_coupling_init(N_MOTORS, icmatrix);

   rc_channels_init(&rc_channels, channel_mapping, channel_scale);

   if (!override_hw)
   {
      LOG(LL_INFO, "initializing i2c bus");
      THROW_ON_ERR(i2c_bus_open(&i2c_4, "/dev/i2c-4"));
      plat->priv = &i2c_4;

      LOG(LL_INFO, "initializing MARG sensor cluster");
      THROW_ON_ERR(freeimu_04_init(&marg, &i2c_4));
      plat->read_marg = read_marg;
     
      plat->read_ultra = ultra_dummy_read;

      LOG(LL_INFO, "initializing ms5611 barometric pressure sensor");
      THROW_ON_ERR(ms5611_reader_init(&i2c_4));
      plat->read_baro = ms5611_reader_get_alt;
   
      /* set-up gps driver: */
      scl_gps_init();
      plat->read_gps = scl_gps_read;

      /* set-up dsl reader: */
      LOG(LL_INFO, "initializing remote control reader");
      if (scl_rc_init() < 0)
      {
         LOG(LL_ERROR, "could not initialize remote control reader");
         exit(1);
      }
      plat->read_rc = read_rc;

      if (scl_power_init() < 0)
      {
         LOG(LL_ERROR, "could not initialize power reader");
         exit(1);
      }
      plat->read_power = scl_power_read;
 
      /* set-up arduino interface */
      LOG(LL_INFO, "Initializing arduino bridge to escs");
      if (arduino_escs_init() < 0)
      {
      	LOG(LL_ERROR, "could not initialize arduino interface");
	      exit(1);
      }
      plat->write_motors = arduino_escs_write;
      ac_init(&plat->ac, 0.1f, 0.7f, 12.0f, 17.0f, c, 4, force_to_esc, 0.0f);
   }

   LOG(LL_INFO, "exynos_quadro platform initialized");
   THROW_END();
}

