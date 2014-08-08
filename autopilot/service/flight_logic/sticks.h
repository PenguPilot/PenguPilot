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


#include "../util/math/vec2.h"

#include <stdbool.h>

#include <threadsafe_types.h>


void sticks_init(void); /* reads the sticks configuration */

float sticks_rotation(void); /* rotation of pitch/roll stick inputs with respect to device frame in rad, configured in deg/s */

float stick_dz(float g, float d);

float sticks_pitch_roll_speed_func(float stick);

float sticks_pitch_roll_angle_func(float stick);

float sticks_gas_acc_func(float stick);

float sticks_gas_speed_func(float stick);

void sticks_pitch_roll_gps_speed_func(vec2_t *speeds, const vec2_t *sticks);

bool sticks_pitch_roll_in_deadzone(float pitch, float roll);

bool sticks_gas_in_deadzone(float gas);

float sticks_gas_speed_deadzone(float gas);

float sticks_yaw_speed_deadzone(float yaw);


#endif /* __STICKS_H__ */

