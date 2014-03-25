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
  
 Main Loop Interface

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __MAIN_LOOP_H__
#define __MAIN_LOOP_H__

#include <stdint.h>
#include <stdbool.h>
#include "../platform/platform.h"


#define REALTIME_PERIOD (0.006)


#define DATA_DEFINITION() \
   float channels[MAX_CHANNELS]; \
   marg_data_t marg_data; \
   float dt; \
   float ultra_z; \
   float baro_z; \
   float voltage; \
   float current; \
   gps_data_t gps_data; \
   uint16_t sensor_status


typedef struct
{
   float pitch;
   float roll;
   float yaw;
   float gas;
}
stick_t;


void main_init(int argc, char *argv[]);

void main_step(const float dt,
               const marg_data_t *marg_data,
               const gps_data_t *gps_data,
               const float ultra,
               const float baro,
               const float voltage,
               const float current,
               const float channels[MAX_CHANNELS],
               const uint16_t sensor_status,
               const bool override_hw);

void main_calibrate(int enabled);

#endif /* __MAIN_LOOP_H__ */

