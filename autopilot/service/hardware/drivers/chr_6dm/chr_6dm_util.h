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
  
 CHR-6DM Utility Functions Interface

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __CHR6DM_UTIL_H__
#define __CHR6DM_UTIL_H__

#define CHR6DM_DIMENSIONS (3) /* x,y,z */

/*
 * channel definitions:
 */
typedef enum
{
   CHR6DM_ACC_Z,
   CHR6DM_ACC_Y,
   CHR6DM_ACC_X,
   CHR6DM_GYRO_Z,
   CHR6DM_GYRO_Y,
   CHR6DM_GYRO_X,
   CHR6DM_MAG_Z,
   CHR6DM_MAG_Y,
   CHR6DM_MAG_X,
   CHR6DM_ROLL_RATE,
   CHR6DM_PITCH_RATE,
   CHR6DM_YAW_RATE,
   CHR6DM_ROLL,
   CHR6DM_PITCH,
   CHR6DM_YAW
}
chr6dm_channel_t;


#define CHR6DM_N_CHANNELS (CHR6DM_YAW + 1)

#define CHR6DM_CHANNEL_TO_BIT(c) (1 << ((c) + 1))

/*
 * channel bitmask definitions:
 */
#define CHR6DM_ACC_Z_BIT      CHR6DM_CHANNEL_TO_BIT(CHR6DM_ACC_Z)
#define CHR6DM_ACC_Y_BIT      CHR6DM_CHANNEL_TO_BIT(CHR6DM_ACC_Y)
#define CHR6DM_ACC_X_BIT      CHR6DM_CHANNEL_TO_BIT(CHR6DM_ACC_X)
#define CHR6DM_GYRO_Z_BIT     CHR6DM_CHANNEL_TO_BIT(CHR6DM_GYRO_Z)
#define CHR6DM_GYRO_Y_BIT     CHR6DM_CHANNEL_TO_BIT(CHR6DM_GYRO_Y)
#define CHR6DM_GYRO_X_BIT     CHR6DM_CHANNEL_TO_BIT(CHR6DM_GYRO_X)
#define CHR6DM_MAG_Z_BIT      CHR6DM_CHANNEL_TO_BIT(CHR6DM_MAG_Z)
#define CHR6DM_MAG_Y_BIT      CHR6DM_CHANNEL_TO_BIT(CHR6DM_MAG_Y)
#define CHR6DM_MAG_X_BIT      CHR6DM_CHANNEL_TO_BIT(CHR6DM_MAG_X)
#define CHR6DM_ROLL_RATE_BIT  CHR6DM_CHANNEL_TO_BIT(CHR6DM_ROLL_RATE)
#define CHR6DM_PITCH_RATE_BIT CHR6DM_CHANNEL_TO_BIT(CHR6DM_PITCH_RATE)
#define CHR6DM_YAW_RATE_BIT   CHR6DM_CHANNEL_TO_BIT(CHR6DM_YAW_RATE)
#define CHR6DM_ROLL_BIT       CHR6DM_CHANNEL_TO_BIT(CHR6DM_ROLL)
#define CHR6DM_PITCH_BIT      CHR6DM_CHANNEL_TO_BIT(CHR6DM_PITCH)
#define CHR6DM_YAW_BIT        CHR6DM_CHANNEL_TO_BIT(CHR6DM_YAW)

#define CHR6DM_ALL_ACCS         (CHR6DM_ACC_Z_BIT | CHR6DM_ACC_Y_BIT | CHR6DM_ACC_X_BIT)
#define CHR6DM_ALL_GYROS        (CHR6DM_GYRO_Z_BIT | CHR6DM_GYRO_Y_BIT | CHR6DM_GYRO_X_BIT)
#define CHR6DM_ALL_MAGS         (CHR6DM_MAG_Z_BIT | CHR6DM_MAG_Y_BIT | CHR6DM_MAG_X_BIT)
#define CHR6DM_ALL_ANGLE_RATES  (CHR6DM_ROLL_RATE_BIT | CHR6DM_PITCH_RATE_BIT | CHR6DM_YAW_RATE_BIT)
#define CHR6DM_ALL_ANGLES       (CHR6DM_ROLL_BIT | CHR6DM_PITCH_BIT | CHR6DM_YAW_BIT)

#define CHR6DM_ALL_CHANNELS (CHR6DM_ALL_ACCS | CHR6DM_ALL_GYROS | CHR6DM_ALL_MAGS | CHR6DM_ALL_ANGLE_RATES | CHR6DM_ALL_ANGLES)
#define CHR6DM_AUTOPILOT_CHANNELS (CHR6DM_ALL_ACCS | CHR6DM_ALL_ANGLES | CHR6DM_ALL_ANGLE_RATES)

float chr6dm_scale_table_entry(chr6dm_channel_t channel);

const char *chr6dm_channel_name(chr6dm_channel_t channel);


#endif /* __CHR6DM_UTIL_H__ */

