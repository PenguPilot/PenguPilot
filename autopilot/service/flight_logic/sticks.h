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
  
 R/C Sticks Config/Util Interface

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __STICKS_H__
#define __STICKS_H__


#include <threadsafe_types.h>


void sticks_init(void); /* reads the sticks configuration */

float sticks_pitch_roll_speed_max(void); /* maximum pitch/roll angle in rad/s, configured in deg/s */

float sticks_pitch_roll_angle_max(void); /* maximum pitch/roll angle in rad, configured in deg*/

float sticks_vert_speed_max(void); /* maximal vertical speed (ultra/baro) in m/s */

float sticks_vert_deadzone(void); /* gps, attitude, auto take-over */

float sticks_horiz_speed_max(void); /* maximal horizontal speed (gps) in m/s */

float sticks_horiz_deadzone(void); /* baro pos, ultra pos, speed in m/s */

float sticks_yaw_speed_max(void); /* maximum yaw rotation speed in rad/s, configured in deg/s */

float sticks_gas_acc_max(void); /* maximum acceleration (m/s^2) for direct gas control */

float sticks_rotation(void); /* rotation of pitch/roll stick inputs with respect to device frame in rad, configured in deg/s */

float stick_dz(float g, float d); /* stick deadzone computation */



#endif /* __STICKS_H__ */

