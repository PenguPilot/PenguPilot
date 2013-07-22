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
  
 vector transformations implementation

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <math.h>

#include "transform.h"


void transform_local_global(vec3_t *gv, vec3_t *lv, quat_t *quat)
{
   /* rotate orientation "right": */
   quat_t zrot_quat;
   quat_init_axis(&zrot_quat, 0.0, 0.0, 1.0, M_PI / 2.0f);
   quat_t tq;
   quat_mul(&tq, &zrot_quat, quat);
   /* rotate vector according to new quaternion: */
   vec3_t tv;
   quat_rot_vec(&tv, lv, &tq);
   tv.y *= -1.0f;
   tv.z *= -1.0f;
   *gv = tv;
}

