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


#ifndef __MAXSONAR_H__
#define __MAXSONAR_H__


#include <stdint.h>


struct maxsonar;
typedef struct maxsonar maxsonar_t;


/*
 * creates a maxsonar protocol parser
 */
maxsonar_t *maxsonar_create(void);


/*
 * executes the maxsonar protocol parser
 * returns -1, if parser failed
 *          0, if parser worked
 *          1, if a new packed was parsed
 */
int maxsonar_parse(maxsonar_t *sonar, uint8_t b);


/*
 * returns the current distance parsed
 */
float maxsonar_get_dist(maxsonar_t *sonar);


#endif /* __MAXSONAR_H__ */

