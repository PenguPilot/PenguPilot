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

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

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


int platform_read_marg(marg_data_t *marg_data)
{
   CHECK_DEV(platform.read_marg);
   return platform.read_marg(marg_data);
}


int platform_read_rc(float channels[MAX_CHANNELS])
{
   CHECK_DEV(platform.read_rc);
   return platform.read_rc(channels);
}


int platform_read_gps(gps_data_t *gps_data)
{
   CHECK_DEV(platform.read_gps);
   return platform.read_gps(gps_data);
}


int platform_read_ultra(float *ultra)
{
   CHECK_DEV(platform.read_ultra);
   return platform.read_ultra(ultra);
}


int platform_read_baro(float *baro)
{
   CHECK_DEV(platform.read_baro);
   return platform.read_baro(baro);
}


int platform_read_voltage(float *voltage)
{
   CHECK_DEV(platform.read_voltage);
   return platform.read_voltage(voltage);
}


int platform_ac_calc(float *setpoints, const int enabled, const float voltage, const float *forces)
{
   return ac_calc(setpoints, &platform.ac, enabled, voltage, forces);
}


uint16_t platform_read_sensors(marg_data_t *marg_data, gps_data_t *gps_data, float *ultra, float *baro, float *voltage, float channels[MAX_CHANNELS])
{
   uint16_t status = 0;
   if (platform_read_marg(marg_data) == 0)
   {
      status |= MARG_VALID;
   }
   
   if (platform_read_gps(gps_data) == 0)
   {
      if (gps_data->fix >= FIX_2D)
      {
         status |= GPS_VALID;
      }
   }
   
   if (platform_read_ultra(ultra) == 0)
   {
      status |= ULTRA_VALID;
   }
    
   if (platform_read_baro(baro) == 0)
   {
      status |= BARO_VALID;
   }
   
   if (platform_read_voltage(voltage) == 0)
   {
      status |= VOLTAGE_VALID;
   }
   
   if (platform_read_rc(channels) == 0)
   {
      status |= RC_VALID;
   }
   
   return status;
}


int platform_write_motors(float *setpoints)
{
   CHECK_DEV(platform.write_motors);
   return platform.write_motors(setpoints);
}

