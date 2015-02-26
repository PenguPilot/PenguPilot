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


int platform_read_marg(marg_data_t *marg_data)
{
   CHECK_DEV(platform.read_marg);
   return platform.read_marg(marg_data);
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


uint8_t platform_read_sensors(marg_data_t *marg_data,
                              float *ultra, 
                              float *baro)
{
   uint8_t status = 0;
   if (platform_read_marg(marg_data) == 0)
   {
      status |= MARG_VALID;
   }
   
   if (platform_read_ultra(ultra) == 0)
   {
      status |= ULTRA_VALID;
   }
    
   if (platform_read_baro(baro) == 0)
   {
      status |= BARO_VALID;
   }
   
   return status;
}

