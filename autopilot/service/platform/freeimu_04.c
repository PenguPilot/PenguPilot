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
  
 FreeIMU V0.4 Driver Implementation

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

#include "freeimu_04.h"


int freeimu_04_init(freeimu_04_t *freeimu, i2c_bus_t *bus)
{
   THROW_BEGIN();
   THROW_ON_ERR(mpu6050_init(&freeimu->mpu, bus, 0x68, MPU6050_DLPF_CFG_94_98Hz, MPU6050_FS_SEL_1000, MPU6050_AFS_SEL_4G));
   THROW_ON_ERR(hmc5883_init(&freeimu->hmc, bus));
   THROW_END();
}


int freeimu_04_read(marg_data_t *data, freeimu_04_t *freeimu)
{
   THROW_BEGIN();
   THROW_ON_ERR(mpu6050_read(&freeimu->mpu, &data->gyro, &data->acc, NULL));
   THROW_ON_ERR(hmc5883_read_mag(&data->mag, &freeimu->hmc));
   THROW_END();
}


