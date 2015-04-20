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
#include <malloc.h>
#include <math.h>

#include <util.h>
#include <i2c/i2c.h>
#include <logger.h>

#include "overo_quad.h"
#include "platform.h"
#include "drotek_marg2.h"
#include "../sensors/i2cxl/i2cxl_reader.h"
#include "../sensors/ms5611/ms5611_reader.h"


#define N_MOTORS 4


static i2c_bus_t i2c_3;
static drotek_marg2_t marg;


static int read_marg(marg_data_t *marg_data)
{
   return drotek_marg2_read(marg_data, &marg);   
}


int overo_quad_init(platform_t *plat)
{
   ASSERT_ONCE();
   THROW_BEGIN();

   LOG(LL_INFO, "initializing i2c bus");
   THROW_ON_ERR(i2c_bus_open(&i2c_3, "/dev/i2c-3"));
   plat->priv = &i2c_3;

   LOG(LL_INFO, "initializing MARG sensor cluster");
   THROW_ON_ERR(drotek_marg2_init(&marg, &i2c_3));
   plat->read_marg = read_marg;

   LOG(LL_INFO, "initializing i2cxl sonar sensor");
   THROW_ON_ERR(i2cxl_reader_init(&i2c_3));
      
   LOG(LL_INFO, "initializing ms5611 barometric pressure sensor");
   THROW_ON_ERR(ms5611_reader_init(&i2c_3));
   
   LOG(LL_INFO, "overo_quad sensor platform initialized");
   THROW_END();
}

