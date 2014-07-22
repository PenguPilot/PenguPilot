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
  
 Generic Platform Implementation

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
#include "platform.h"
#include "../sensors/scl_gps/scl_gps.h"
#include "../sensors/scl_power/scl_power.h"
#include "../sensors/scl_rc/scl_rc.h"
#include "../sensors/util/rc_channels.h"
#include "../util/logger/logger.h"


/* pitch: 0, roll: 1, yaw: 3, gas: 2, switch left: 4, switch right: 5 */
static uint8_t channel_mapping[PP_MAX_CHANNELS] =  {0, 1, 3, 2, 4, 5}; 
static float channel_scale[PP_MAX_CHANNELS] =  {1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f};
static rc_channels_t rc_channels;


static int read_rc(float channels[PP_MAX_CHANNELS])
{
   float dsl_channels[PP_MAX_CHANNELS];
   int ret = scl_rc_read(dsl_channels);
   for (int c = 0; c < PP_MAX_CHANNELS; c++)
      channels[c] = rc_channels_get(&rc_channels, dsl_channels, c);
   return ret;
}


int generic_platform_init(platform_t *plat)
{
   ASSERT_ONCE();
   THROW_BEGIN();
   LOG(LL_INFO, "initializing power reader");
   THROW_ON_ERR(scl_power_init());
   plat->read_power = scl_power_read;
   
   LOG(LL_INFO, "initializing remote control reader");
   rc_channels_init(&rc_channels, channel_mapping, channel_scale);
   THROW_ON_ERR(scl_rc_init());
   plat->read_rc = read_rc;
 
   LOG(LL_INFO, "initializing GPS reader");
   THROW_ON_ERR(scl_gps_init());
   plat->read_gps = scl_gps_read;

   THROW_END();
}

