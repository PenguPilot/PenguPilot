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
  
 Attitude and Thrust Computation Interface

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __ATT_THRUST_H__
#define __ATT_THRUST_H__


#include "../geometry/vec3.h"
#include "../util/math/vec2.h"


/* n,e,d force to attitude and thrust conversion */
int att_thrust_calc(vec2_t *ne_out, float *thrust, /* output: angle north, angle east, thrust */
                    vec3_t *f_ned,   /* input of n,e,d forces */
                    float thrust_max, /* maximum thrust force */
                    int update_f_ned  /* if true, f_ned is updated if thrust_max has been exceeded */);


#endif /* __ATT_THRUST_H__ */

