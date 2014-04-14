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
  
 Quaternion Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology
 Most of the code was borrowed from the Internet

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <string.h>
#include <math.h>
#include <util.h>

#include "conv.h"
#include "quat.h"


void quat_init(quat_t *q)
{
   q->dim = 4;
   q->ve = &q->data[0];
   q->x = REAL(1.0);
   q->y = REAL(0.0);
   q->z = REAL(0.0);
   q->w = REAL(0.0);
}


void quat_init_data(quat_t *q, real_t x, real_t y, real_t z, real_t w)
{
   q->dim = 4;
   q->ve = &q->data[0];
   q->x = x;
   q->y = y;
   q->z = z;
   q->w = w;
}


void quat_init_axis(quat_t *q, real_t x, real_t y, real_t z, real_t a)
{
   /* see: http://www.euclideanspace.com/maths/geometry/rotations
           /conversions/angleToQuaternion/index.htm */
   real_t a2 = a * 0.5f;
   real_t s = sin(a2);
   quat_init_data(q, x * s, y * s, z * s, cos(a2));
}


void quat_init_axis_v(quat_t *q, const vec3_t *v, real_t a)
{
   quat_init_axis(q, v->x, v->y, v->z, a);
}


void quat_normalize(quat_t *q)
{
   vec_normalize(q);
}


void quat_rot_vec(vec3_t *vo, const vec3_t *vi, const quat_t *q)
{
   /* see: https://github.com/qsnake/ase/blob/master/ase/quaternions.py */
   const real_t vx = vi->x, vy = vi->y, vz = vi->z;
   const real_t qw = q->w, qx = q->x, qy = q->y, qz = q->z;
   const real_t qww = qw * qw, qxx = qx * qx, qyy = qy * qy, qzz = qz * qz;
   const real_t qwx = qw * qx, qwy = qw * qy, qwz = qw * qz, qxy = qx * qy;
   const real_t qxz = qx * qz, qyz = qy * qz;
   vo->x = (qww + qxx - qyy - qzz) * vx + 2 * ((qxy - qwz) * vy + (qxz + qwy) * vz);
   vo->y = (qww - qxx + qyy - qzz) * vy + 2 * ((qxy + qwz) * vx + (qyz - qwx) * vz);
   vo->z = (qww - qxx - qyy + qzz) * vz + 2 * ((qxz - qwy) * vx + (qyz + qwx) * vy);
}


void quat_conj(quat_t *q_out, const quat_t *q_in)
{
   q_out->x = -q_in->x;
   q_out->y = -q_in->y;
   q_out->z = -q_in->z;
   q_out->w = q_in->w;
}


void quat_to_euler(euler_t *euler, const quat_t *quat)
{
   const real_t x = quat->x, y = quat->y, z = quat->z, w = quat->w;
   const real_t ww = w * w, xx = x * x, yy = y * y, zz = z * z;
   euler->yaw = norm_angle_0_2pi(atan2f(2.f * (x * y + z * w), xx - yy - zz + ww));
   euler->pitch = asinf(-2.f * (x * z - y * w));
   euler->roll = atan2f(2.f * (y * z + x * w), -xx - yy + zz + ww);
}


void quat_mul(quat_t *o, const quat_t *q1, const quat_t *q2)
{
   /* see: http://www.euclideanspace.com/maths/algebra/
           realNormedAlgebra/quaternions/code/index.htm#mul */
   o->x =  q1->x * q2->w + q1->y * q2->z - q1->z * q2->y + q1->w * q2->x;
   o->y = -q1->x * q2->z + q1->y * q2->w + q1->z * q2->x + q1->w * q2->y;
   o->z =  q1->x * q2->y - q1->y * q2->x + q1->z * q2->w + q1->w * q2->z;
   o->w = -q1->x * q2->x - q1->y * q2->y - q1->z * q2->z + q1->w * q2->w;
}

