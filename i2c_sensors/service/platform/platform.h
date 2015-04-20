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

#include <marg_data.h>


typedef struct
{
   /* sensors: */
   int (*read_marg)(marg_data_t *marg_data);
   int (*read_ultra)(float *ultra);
   int (*read_baro)(float *baro);
   /* private data: */
   void *priv;
}
platform_t;


extern platform_t platform;


int platform_read_marg(marg_data_t *marg_data);


int platform_read_ultra(float *ultra);


int platform_read_baro(float *baro);


#define GYRO_VALID    0x01
#define ACC_VALID     0x02
#define MAG_VALID     0x04
#define ULTRA_VALID   0x08
#define BARO_VALID    0x10
#define MARG_VALID    (GYRO_VALID | ACC_VALID | MAG_VALID)
#define SENSORS_VALID (MARG_VALID | ULTRA_VALID | BARO_VALID)


uint8_t platform_read_sensors(marg_data_t *marg_data);


#endif /* __PLATFORM_H__ */

