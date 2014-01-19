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
  
 CHR-6DM Driver Interface

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __CHR6DM_H__
#define __CHR6DM_H__


typedef struct
{
   /* euler angles: */
   float pitch; /* -PI .. PI, 0 is horizontal */
   float roll; /* -PI .. PI, 0 is horizontal */
   float yaw; /* -PI .. PI, 0 is north */

   /* angular speeds: */
   float pitch_rate; /* in rad / s */
   float roll_rate; /* in rad / s */
   float yaw_rate;  /* in rad / s */

   /* acc acceleration: */
   float acc_pitch; /* in m / (s ^ 2) */
   float acc_roll; /* in m / (s ^ 2) */
   float acc_yaw;  /* in rad / (s ^ 2) */
}
ahrs_data_t;


int chr6dm_init(void);

int chr6dm_read(ahrs_data_t *data);

void chr6dm_wait_for_data(void);


#endif /* __CHR6DM_H__ */

