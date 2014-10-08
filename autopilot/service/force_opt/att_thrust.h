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

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

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


#include "../util/math/vec2.h"
#include "../util/math/vec3.h"


int att_thrust_calc(vec2_t *pr_angles, float *thrust, /* output: pitch/roll angles + thrust */
                    vec3_t *f_neu,   /* input of p,r,d forces */
                    float yaw, /* yaw orientation */
                    float thrust_max, /* maximum thrust force */
                    int update_f_neu  /* if true, f_prd is updated if thrust_max has been exceeded */);


#endif /* __ATT_THRUST_H__ */

