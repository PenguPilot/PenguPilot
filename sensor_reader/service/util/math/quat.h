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
  
 Quaternion Interface

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau
 Most of the code was borrowed from the Internet

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __QUAT_H__
#define __QUAT_H__


#include "vec3.h"
#include "euler.h"


typedef struct
{
   size_t dim;
   real_t *ve;
   union {
      real_t data[4];
      struct
      {
         real_t q0;
         real_t q1;
         real_t q2;
         real_t q3;
      };
      struct
      {
         real_t w;
         real_t x;
         real_t y;
         real_t z;
      };
   };
}
quat_t;


/* init quaternion to (x, y, z, w) = (0, 0, 0, 1) */
void quat_init(quat_t *q);

/* init quaternion from data */
void quat_init_data(quat_t *q, real_t x, real_t y, real_t z, real_t w);
 
/* initialize quaternion from axis angle using argument values */
void quat_init_axis(quat_t *q, real_t x, real_t y, real_t z, real_t a);

/* initialize quaternion from axis angle using a vector */
void quat_init_axis_v(quat_t *q, const vec3_t *v, real_t a);

/* normalize quaternion */
void quat_normalize(quat_t *q);

/* rotate vector vi via unit quaternion q and put result into vector vo */
void quat_rot_vec(vec3_t *vo, const vec3_t *vi, const quat_t *q);

/* rotate vector v_in in-place via unit quaternion quat */
void quat_rot_vec_self(vec3_t *v, const quat_t *q);

/* conjugate quaternion */
void quat_conj(quat_t *qo, const quat_t *qi);

/* o = q1 * q2 */
void quat_mul(quat_t *o, const quat_t *q1, const quat_t *q2);

/* convert quaternion to euler angles */
void quat_to_euler(euler_t *e, const quat_t *q);


#endif /* __QUAT_H__ */

