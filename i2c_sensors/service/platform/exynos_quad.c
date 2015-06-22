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
#include "../sensors/ak8975c/ak8975c.h"
#include "../sensors/mpu6050/mpu6050.h"
#include "../sensors/i2cxl/i2cxl.h"
#include "../sensors/ms5611/ms5611.h"



static i2c_bus_t i2c_bus;
static mpu6050_t mpu;
static ak8975c_t ak;
static i2cxl_t i2cxl;
static ms5611_t ms5611;
static float gyro_temperature;
static vec3_t acc;
static int ret;
static quat_t q;


static int read_gyro(vec3_t *gyro)
{
   THROW_BEGIN();
   vec3_t _gyro;
   vec3_init(&_gyro);
   ret = mpu6050_read(&mpu, &_gyro, &acc, &gyro_temperature);
   THROW_ON_ERR(ret);
   quat_rot_vec(gyro, &_gyro, &q);
   THROW_END();
}


static int read_acc(vec3_t *acc_out)
{
   THROW_BEGIN();
   THROW_ON_ERR(ret);
   quat_rot_vec(acc_out, &acc, &q);
   THROW_END();
}


static int read_mag(vec3_t *mag)
{
   THROW_BEGIN();
   THROW_ON_ERR(ak8975c_read(&ak));
   /* align ak to mpu: */
   ak.raw.z = -ak.raw.z;
   float tmp = ak.raw.x;
   ak.raw.x = ak.raw.y;
   ak.raw.y = tmp;
   /* rotate: */
   quat_rot_vec(mag, &ak.raw, &q);
   THROW_END();
}


static int read_ultra(float *altitude)
{
   THROW_BEGIN();
   THROW_ON_ERR(i2cxl_read(&i2cxl, altitude));
   THROW_END();
}


static int read_baro(float *altitude, float *temperature)
{
   THROW_BEGIN();
   THROW_ON_ERR(ms5611_measure(&ms5611));
   *altitude = ms5611.c_a;
   *temperature = ms5611.c_t;
   THROW_END();
}


int exynos_quad_init(platform_t *plat)
{
   ASSERT_ONCE();
   THROW_BEGIN();
   vec3_init(&acc);
   quat_init_axis(&q, 0, 0, 1, -3.0 * M_PI / 4.0f);
 
   LOG(LL_INFO, "initializing i2c bus");
   THROW_ON_ERR(i2c_bus_open(&i2c_bus, "/dev/i2c-4"));

   LOG(LL_INFO, "initializing MARG sensor cluster");
   THROW_ON_ERR(mpu6050_init(&mpu, &i2c_bus, 0x69, MPU6050_DLPF_CFG_94_98Hz, MPU6050_FS_SEL_1000, MPU6050_AFS_SEL_4G));
   plat->read_gyro = read_gyro;
   plat->read_acc = read_acc;

   LOG(LL_INFO, "initializing AK8975C");
   THROW_ON_ERR(ak8975c_init(&ak, &i2c_bus));
   plat->read_mag = read_mag;
   
   LOG(LL_INFO, "initializing i2cxl sonar sensor");
   THROW_ON_ERR(i2cxl_init(&i2cxl, &i2c_bus));
   plat->read_ultra = read_ultra;

   LOG(LL_INFO, "initializing ms5611 barometric pressure sensor");
   THROW_ON_ERR(ms5611_init(&ms5611, &i2c_bus, MS5611_OSR4096, MS5611_OSR4096));
   plat->read_baro = read_baro;

   LOG(LL_INFO, "exynos_quadro sensor platform initialized");
   THROW_END();
}

