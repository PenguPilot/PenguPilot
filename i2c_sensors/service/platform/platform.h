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
  
 Platform Abstraction Interface

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __PLATFORM_H__
#define __PLATFORM_H__


#include <stdint.h>

#include <math/vec3.h>


typedef struct
{
   int (*read_gyro)(vec3_t *gyro);
   int (*read_acc)(vec3_t *acc);
   int (*read_mag)(vec3_t *mag);
   int (*read_ultra)(float *altitude);
   int (*read_baro)(float *altitude, float *temperature);
}
platform_t;


extern platform_t platform;


int platform_read_gyro(vec3_t *gyro);

int platform_read_acc(vec3_t *acc);

int platform_read_mag(vec3_t *mag);

int platform_read_ultra(float *altitude);

int platform_read_baro(float *altitude, float *temperature);


#endif /* __PLATFORM_H__ */

