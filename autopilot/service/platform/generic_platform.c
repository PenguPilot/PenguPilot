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
#include "../util/logger/logger.h"
#include "../channels/channels.h"


static int read_rc(float channels[PP_MAX_CHANNELS])
{
   float all_channels[MAX_CHANNELS];
   int ret = scl_rc_read(all_channels);
   channels_update(channels, all_channels);
   return ret;
}


int generic_platform_init(platform_t *plat)
{
   ASSERT_ONCE();
   THROW_BEGIN();
   LOG(LL_INFO, "initializing power reader");
   THROW_ON_ERR(scl_power_init());
   plat->read_power = scl_power_read;
   
   LOG(LL_INFO, "initializing remote control channel map/calibration");
   THROW_ON_ERR(channels_init());
   
   LOG(LL_INFO, "initializing remote control reader");
   THROW_ON_ERR(scl_rc_init());
   plat->read_rc = read_rc;
 
   LOG(LL_INFO, "initializing GPS reader");
   THROW_ON_ERR(scl_gps_init());
   plat->read_gps = scl_gps_read;

   THROW_END();
}

