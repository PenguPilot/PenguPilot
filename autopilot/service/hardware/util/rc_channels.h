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
  
 File Purpose

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __RC_CHANNELS_H__
#define __RC_CHANNELS_H__


#include <stdint.h>

#include "deadzone.h"


#define MAX_CHANNELS 6


typedef enum
{
   CH_PITCH,
   CH_ROLL,
   CH_YAW,
   CH_GAS,
   CH_SWITCH_L,
   CH_SWITCH_R
}
channel_t;


typedef struct
{
   uint8_t *map;
   float *scale;
   deadzone_t *deadzone;
}
rc_channels_t;


void rc_channels_init(rc_channels_t *channels, uint8_t map[MAX_CHANNELS], float scale[MAX_CHANNELS], deadzone_t *deadzone);

float rc_channels_get(rc_channels_t *channels, float *raw_channels, channel_t channel);


#endif /* __RC_CHANNELS__ */

