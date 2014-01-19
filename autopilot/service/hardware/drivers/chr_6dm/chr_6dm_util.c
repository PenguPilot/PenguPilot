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
  
 CHR-6DM Utility Functions Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <math.h>
#include "chr_6dm_util.h"


static const float CHR6DM_SCALE_TABLE[CHR6DM_N_CHANNELS / CHR6DM_DIMENSIONS] =
{
   0.106812,  /* acc  x / y / z */
   0.01812,   /* gyro x / y / z */
   0.061035,  /* mag  x / y / z */
   0.0137329 * M_PI / 180.0, /* yaw / pitch / roll angle rate */
   0.0109863  /* yaw / pitch / roll angle */
};


static const char *CHR6DM_CHANNEL_NAMES[CHR6DM_N_CHANNELS] =
{
   "acc_z",
   "acc_y",
   "acc_x",
   "gyro_z",
   "gyro_y",
   "gyro_x",
   "mag_z",
   "mag_y",
   "mag_x",
   "roll_rate",
   "pitch_rate",
   "yaw_rate",
   "roll_angle",
   "pitch_angle",
   "yaw_angle"
};



float chr6dm_scale_table_entry(chr6dm_channel_t channel)
{
   return CHR6DM_SCALE_TABLE[channel / CHR6DM_DIMENSIONS];
}


const char *chr6dm_channel_name(chr6dm_channel_t channel)
{
   return CHR6DM_CHANNEL_NAMES[channel];
}

