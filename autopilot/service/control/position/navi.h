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
  
 Navigation Interface

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology
 Copyright (C) 2013 Jan Roemisch, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __NAVI_H__
#define __NAVI_H__


#include "../../util/math/vec2.h"


/*
 * allocates and initializes memory for navigation control subsystem
 */
void navi_init(void);


/*
 * resets i-part(s) of the navigation algorithm
 */
void navi_reset(void);


/*
 * setter for e position
 */
void navi_set_dest_e(float x);


/*
 * setter for n position
 */
void navi_set_dest_n(float y);


void navi_set_dest(vec2_t vec);


/*
 * getter for e position
 */
float navi_get_dest_e(void);


/*
 * getter for n position
 */
float navi_get_dest_n(void);


/*
 * set travel speed to standard
 */
void navi_reset_travel_speed(void);


/*
 * set navi travel speed
 */
int navi_set_travel_speed(float speed);


/*
 * executes navigation control subsystem
 */
void navi_run(vec2_t *speed_setpoint, vec2_t *pos, float dt);


#endif /* __NAVI_H__ */

