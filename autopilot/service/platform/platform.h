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

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

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

#include "../hardware/util/gps_data.h"
#include "../hardware/util/rc_channels.h"
#include "../hardware/util/marg_data.h"
#include "../geometry/quat.h"
#include "inv_coupling.h"
#include "ac.h"


typedef struct
{
   /* parameters: */
   float imtx1;
   float imtx2;
   float imtx3;
   float rpm_square_min;
   float rpm_square_max;
   ac_t ac;
   float max_thrust_n;
   float mass_kg;
   inv_coupling_t inv_coupling;
   size_t n_motors;
   
   /* sensors: */
   int (*read_marg)(marg_data_t *marg_data);
   int (*read_rc)(float channels[MAX_CHANNELS]);
   int (*read_gps)(gps_data_t *gps_data);
   int (*read_ultra)(float *ultra);
   int (*read_baro)(float *baro);
   int (*read_power)(float *voltage, float *current);
   
   /* actuators: */
   int (*write_motors)(float *setpoints);

   /* private data: */
   void *priv;
}
platform_t;


extern platform_t platform;


int platform_read_marg(marg_data_t *marg_data);


int platform_read_rc(float channels[MAX_CHANNELS]);


int platform_read_gps(gps_data_t *gps_data);


int platform_read_ultra(float *ultra);


int platform_read_baro(float *baro);


int platform_read_power(float *voltage, float *current);


int platform_ac_calc(float *setpoints, const int enabled, const float voltage, const float *forces);


#define GYRO_VALID    0x01
#define ACC_VALID     0x02
#define MAG_VALID     0x04
#define GPS_VALID     0x08
#define ULTRA_VALID   0x10
#define BARO_VALID    0x20
#define POWER_VALID   0x40
#define RC_VALID      0x80
#define MARG_VALID    (GYRO_VALID | ACC_VALID | MAG_VALID)
#define SENSORS_VALID (MARG_VALID | GPS_VALID | ULTRA_VALID | BARO_VALID | POWER_VALID | RC_VALID)


uint16_t platform_read_sensors(marg_data_t *marg_data,
                               gps_data_t *gps_data,
                               float *ultra,
                               float *baro,
                               float *voltage,
                               float *current,
                               float channels[MAX_CHANNELS]);


int platform_write_motors(float *setpoints);


#endif /* __PLATFORM_H__ */

