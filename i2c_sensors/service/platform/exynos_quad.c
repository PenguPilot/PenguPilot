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


#include <stddef.h>
#include <unistd.h>
#include <math.h>

#include <logger.h>
#include <util.h>
#include <i2c/i2c.h>
#include <math/quat.h>

#include "platform.h"
#include "drotek_9150.h"
#include "../sensors/i2cxl/i2cxl_reader.h"
#include "../sensors/ms5611/ms5611_reader.h"


static i2c_bus_t i2c_4;
static drotek_9150_t marg;


static int read_marg(marg_data_t *marg_data)
{
   int ret = drotek_9150_read(marg_data, &marg);
   quat_t q;
   quat_init_axis(&q, 0, 0, 1, -3.0 * M_PI / 4.0f);
   quat_rot_vec(&marg_data->acc, &marg_data->acc, &q);
   quat_rot_vec(&marg_data->gyro, &marg_data->gyro, &q);
   quat_rot_vec(&marg_data->mag, &marg_data->mag, &q);
   return ret;
}


int exynos_quad_init(platform_t *plat)
{
   ASSERT_ONCE();
   THROW_BEGIN();

   LOG(LL_INFO, "initializing i2c bus");
   THROW_ON_ERR(i2c_bus_open(&i2c_4, "/dev/i2c-4"));
   plat->priv = &i2c_4;

   LOG(LL_INFO, "initializing MARG sensor cluster");
   THROW_ON_ERR(drotek_9150_init(&marg, &i2c_4));
 
   LOG(LL_INFO, "initializing i2cxl sonar sensor");
   THROW_ON_ERR(i2cxl_reader_init(&i2c_4));

   LOG(LL_INFO, "initializing ms5611 barometric pressure sensor");
   THROW_ON_ERR(ms5611_reader_init(&i2c_4));
   
   LOG(LL_INFO, "exynos_quadro sensor platform initialized");
   THROW_END();
}

