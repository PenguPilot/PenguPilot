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
  
 DROTEK MARG (MPU) Driver Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <util.h>

#include "drotek_marg2.h"


int drotek_marg2_init(drotek_marg2_t *marg2, i2c_bus_t *bus)
{
   THROW_BEGIN();
   THROW_ON_ERR(mpu6050_init(&marg2->mpu, bus, MPU6050_DLPF_CFG_94_98Hz, MPU6050_FS_SEL_2000, MPU6050_AFS_SEL_8G));
   THROW_ON_ERR(hmc5883_init(&marg2->hmc, bus));
   THROW_END();
}


int drotek_marg2_read(marg_data_t *data, drotek_marg2_t *marg2)
{
   THROW_BEGIN();
   THROW_ON_ERR(mpu6050_read(&marg2->mpu, &data->gyro, &data->acc, NULL));
   THROW_ON_ERR(hmc5883_read_mag(data->mag.vec, &marg2->hmc));
   THROW_END();
}


