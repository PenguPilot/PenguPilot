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
  
 Gumstix Overo based Quad-Rotor Platform

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
#include "overo_quad.h"
#include "platform.h"
#include "../util/logger/logger.h"

/* hardware includes: */
#include <i2c/i2c.h>
#include "../hardware/drivers/scl_gps/scl_gps.h"
#include "../hardware/drivers/i2cxl/i2cxl_reader.h"
#include "../hardware/drivers/ms5611/ms5611_reader.h"
#include "../hardware/drivers/scl_rc/scl_rc.h"
#include "../hardware/drivers/scl_power/scl_power.h"
#include "../hardware/drivers/pwm_esc/pwm_escs.h"
#include "../hardware/util/rc_channels.h"
#include "drotek_marg2.h"
#include "force_to_esc.h"


#define N_MOTORS 4


static i2c_bus_t i2c_3;
static rc_channels_t rc_channels;
static uint8_t channel_mapping[MAX_CHANNELS] =  {0, 1, 3, 2, 4, 5}; /* pitch: 0, roll: 1, yaw: 3, gas: 2, switch left: 4, switch right: 5 */
static float channel_scale[MAX_CHANNELS] =  {1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f};
static drotek_marg2_t marg;

/*
 * Quad-Rotor Configuration:
 *
 *        (F)
 *         |
 *         |
 * (L)-----+------(R)
 *         |
 *         |
 *        (R)
 */

static uint8_t motor_addrs[N_MOTORS] = 
{
   10, /* (F)ront */
    9, /* (R)ear */
   11, /* (L)eft */
    8  /* (R)ight */
};



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
   return drotek_marg2_read(marg_data, &marg);   
}


int overo_quad_init(platform_t *plat, int override_hw)
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
      /* m2 */ imtx1,   0.0f,  imtx2, -imtx3,
      /* m3 */ imtx1, -imtx2,   0.0f,  imtx3,
      /* m4 */ imtx1,  imtx2,   0.0f,  imtx3
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
   plat->mass_kg = 1.0f;
   plat->n_motors = N_MOTORS;

   LOG(LL_INFO, "initializing inverse coupling matrix");
   inv_coupling_init(N_MOTORS, icmatrix);

   rc_channels_init(&rc_channels, channel_mapping, channel_scale);

   if (!override_hw)
   {
      LOG(LL_INFO, "initializing i2c bus");
      THROW_ON_ERR(i2c_bus_open(&i2c_3, "/dev/i2c-3"));
      plat->priv = &i2c_3;

      LOG(LL_INFO, "initializing MARG sensor cluster");
      THROW_ON_ERR(drotek_marg2_init(&marg, &i2c_3));
      plat->read_marg = read_marg;

      LOG(LL_INFO, "initializing i2cxl sonar sensor");
      THROW_ON_ERR(i2cxl_reader_init(&i2c_3));
      plat->read_ultra = i2cxl_reader_get_alt;
      
      LOG(LL_INFO, "initializing ms5611 barometric pressure sensor");
      THROW_ON_ERR(ms5611_reader_init(&i2c_3));
      plat->read_baro = ms5611_reader_get_alt;
   
      /* initialize motors: */
      ac_init(&plat->ac, 0.1f, 0.7f, 12.0f, 17.0f, c, N_MOTORS, force_to_esc_setup2, 0.0f);
      pwm_escs_init(motor_addrs, N_MOTORS);
      plat->write_motors = pwm_escs_write;

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
   }

   LOG(LL_INFO, "overo_quadro platform initialized");
   THROW_END();
}

