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


/*
   orientation library - implementation

   Copyright (C) 2012 Tobias Simon
   most of the code was stolen from the Internet

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
*/



#include <string.h>
#include <math.h>
#include <util.h>

#include "orientation.h"


void quat_rot_vec(vec3_t *v_out, const vec3_t *v_in, const quat_t *quat)
{
   float r = quat->q0;
   float i = quat->q1;
   float j = quat->q2;
   float k = quat->q3;
   v_out->x = 2 * (r * v_in->z * j + i * v_in->z * k - r * v_in->y * k + i * v_in->y * j) + v_in->x * (r * r + i * i - j * j - k * k);
   v_out->y = 2 * (r * v_in->x * k + i * v_in->x * j - r * v_in->z * i + j * v_in->z * k) + v_in->y * (r * r - i * i + j * j - k * k);
   v_out->z = 2 * (r * v_in->y * i - r * v_in->x * j + i * v_in->x * k + j * v_in->y * k) + v_in->z * (r * r - i * i - j * j + k * k);
}


void quat_copy(quat_t *q_out, const quat_t *q_in)
{
   memcpy(q_out, q_in, sizeof(quat_t));   
}


void quat_to_euler(euler_t *euler, const quat_t *quat)
{
   float s = quat->q0;
   float x = quat->q1;
   float y = quat->q2;
   float z = quat->q3;

   float sqw = s * s;
   float sqx = x * x;
   float sqy = y * y;
   float sqz = z * z;

   euler->yaw = normalize_euler_0_2pi(atan2f(2.f * (x * y + z * s), sqx - sqy - sqz + sqw));
   euler->pitch = asinf(-2.f * (x * z - y * s));
   euler->roll = atan2f(2.f * (y * z + x * s), -sqx - sqy + sqz + sqw);
}


void quat_mul(quat_t *t, const quat_t *q, const quat_t *r)
{
   quat_t _t;
   _t.vec[0] = r->vec[0] * q->vec[0] - r->vec[1] * q->vec[1] - r->vec[2] * q->vec[2] - r->vec[3] * q->vec[3];
   _t.vec[1] = r->vec[0] * q->vec[1] + r->vec[1] * q->vec[0] - r->vec[2] * q->vec[3] + r->vec[3] * q->vec[2];
   _t.vec[2] = r->vec[0] * q->vec[2] + r->vec[1] * q->vec[3] + r->vec[2] * q->vec[0] - r->vec[3] * q->vec[1];
   _t.vec[3] = r->vec[0] * q->vec[3] - r->vec[1] * q->vec[2] + r->vec[2] * q->vec[1] + r->vec[3] * q->vec[0];
   quat_normalize(&_t);
   *t = _t;
}


void quat_normalize(quat_t *q)
{
   float norm = 0.0f;
   FOR_N(i, 4)
   {
      norm += q->vec[i] * q->vec[i];
   }
   norm = 1.0f / sqrt(norm);
   FOR_N(i, 4)
   {
      q->vec[i] *= norm;
   }
}


float normalize_euler_0_2pi(float euler_angle)
{
   if (euler_angle < 0)
   {
      euler_angle += (float)(2 * M_PI);
   }
   return euler_angle;
}


void quaternion_init(quat_t *quat, vec3_t *acc, vec3_t *mag)
{
   float ax = acc->x;
   float ay = acc->y;
   float az = acc->z;
   float mx = mag->x;
   float my = mag->y;
   float mz = mag->z;


   float init_roll = atan2(-ay, -az);
   float init_pitch = atan2(ax, -az);

   float cos_roll = cosf(init_roll);
   float sin_roll = sinf(init_roll);
   float cos_pitch = cosf(init_pitch);
   float sin_pitch = sinf(init_pitch);

   float mag_x = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
   float mag_y = my * cos_roll - mz * sin_roll;

   float init_yaw = atan2(-mag_y, mag_x);

   cos_roll =  cosf(init_roll * 0.5f);
   sin_roll =  sinf(init_roll * 0.5f);

   cos_pitch = cosf(init_pitch * 0.5f );
   sin_pitch = sinf(init_pitch * 0.5f );

   float cosHeading = cosf(init_yaw * 0.5f);
   float sinHeading = sinf(init_yaw * 0.5f);

   quat->q0 = cos_roll * cos_pitch * cosHeading + sin_roll * sin_pitch * sinHeading;
   quat->q1 = sin_roll * cos_pitch * cosHeading - cos_roll * sin_pitch * sinHeading;
   quat->q2 = cos_roll * sin_pitch * cosHeading + sin_roll * cos_pitch * sinHeading;
   quat->q3 = cos_roll * cos_pitch * sinHeading - sin_roll * sin_pitch * cosHeading;
}



