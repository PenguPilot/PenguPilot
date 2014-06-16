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
  
 Drotek MPU9150 + MS5611 Driver Implementation

 Copyright (C) 2014 Jan Roemisch, Ilmenau University of Technology
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

#include "drotek_9150.h"
#include "../drivers/ak8975c/ak8975c_reader.h"


int drotek_9150_init(drotek_9150_t *drotek, i2c_bus_t *bus)
{
   THROW_BEGIN();
   THROW_ON_ERR(mpu6050_init(&drotek->mpu, bus, 0x69,
      MPU6050_DLPF_CFG_94_98Hz, MPU6050_FS_SEL_1000, MPU6050_AFS_SEL_4G));
   THROW_ON_ERR(ak8975c_reader_init(bus));
   THROW_END();
}


int drotek_9150_read(marg_data_t *data, drotek_9150_t *drotek)
{
   float tmp;

   THROW_BEGIN();
   
   THROW_ON_ERR(mpu6050_read(&drotek->mpu, &data->gyro, &data->acc, NULL));
   THROW_ON_ERR(ak8975c_reader_get(&data->mag));

   data->mag.z = -data->mag.z;
   tmp = data->mag.x;
   data->mag.x = data->mag.y;
   data->mag.y = tmp;
   
   THROW_END();
}


