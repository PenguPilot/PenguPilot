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
  
 Platform Abstraction Implementation

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "platform.h"

#include <string.h>
#include <malloc.h>
#include <assert.h>
#include <errno.h>


platform_t platform;


#define CHECK_DEV(x) \
   if (!x) \
      return -ENODEV


int platform_read_gyro(vec3_t *gyro)
{
   CHECK_DEV(platform.read_gyro);
   return platform.read_gyro(gyro);
}


int platform_read_acc(vec3_t *acc)
{
   CHECK_DEV(platform.read_gyro);
   return platform.read_acc(acc);
}


int platform_read_mag(vec3_t *mag)
{
   CHECK_DEV(platform.read_mag);
   return platform.read_mag(mag);
}


int platform_read_ultra(float *altitude)
{
   CHECK_DEV(platform.read_ultra);
   return platform.read_ultra(altitude);
}


int platform_read_baro(float *altitude, float *temperature)
{
   CHECK_DEV(platform.read_baro);
   return platform.read_baro(altitude, temperature);
}

