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
  
 Kalman Filter based Position/Speed Estimate
   
 | 1 dt | * | p | + | 0.5 * dt ^ 2 | * | a | = | p |
 | 0  1 | * | v |   |     dt       |   | v |
 
 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology
 Copyright (C) 2012 Jan Roemisch, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __POS_H__
#define __POS_H__


#include "../geometry/quat.h"
#include "../util/math/vec2.h"


typedef struct
{
   float pos; /* position, in m */
   float speed; /* in m / s */
}
pos_speed_t;


typedef struct
{
   /* system positions/speeds in NEU ground reference frame: */
   vec2_t xy_pos;
   vec2_t xy_speed;
   pos_speed_t ultra_z; /* ultrasonic altitude altitude above ground */
   pos_speed_t baro_z; /* barometric altitude above MSL */
}
pos_t;


typedef struct
{
   float dt;

   /* positions input: */
   float ultra_z;
   float baro_z;
   float dx;
   float dy;

   /* control acc input in NEU ground reference frame: */
   vec3_t acc;
}
pos_in_t;


/* initializes the position estimate subsystem */
void pos_init(void);


/* computes new position/speed estimates using the given input signals */
void pos_update(pos_t *out, pos_in_t *in);


#endif /* __POS_H__ */

