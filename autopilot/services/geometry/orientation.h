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
   orientation library - interface

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


#ifndef __ORIENTATION_H__
#define __ORIENTATION_H__


float normalize_euler_0_2pi(float euler_angle);


/* generic 3d vector */
typedef union
{
   struct
   {
      float x;
      float y;
      float z;
   };
   float vec[3];
}
vec3_t;


/* quaternion */
typedef union
{
   struct
   {
      float q0;
      float q1;
      float q2;
      float q3;
   };
   float vec[4];
}
quat_t;


/* euler angle */
typedef union
{
   struct
   {
      float yaw;
      float pitch;
      float roll;
   };
   float vec[3];
}
euler_t;


/* rotate vector v_in via unit quaternion quat and
   put result into v_out: */
void quat_rot_vec(vec3_t *v_out, const vec3_t *v_in, const quat_t *quat);


/* copy q_in to q_out: */
void quat_copy(quat_t *q_out, const quat_t *q_in);


/* inverse quaternion: */
void quat_inv(quat_t *q_out, const quat_t *q_in);


/* t = q x r */
void quat_mul(quat_t *t, const quat_t *q, const quat_t *r);


/* normalizes quaternion q */
void quat_normalize(quat_t *q);


/* convert quaternion to euler angles: */
void quat_to_euler(euler_t *euler, const quat_t *quat);


float normalize_euler_0_2pi(float euler_angle);


void quaternion_init(quat_t *quat, vec3_t *acc, vec3_t *mag);


#endif /* __ORIENTATION_H__ */

