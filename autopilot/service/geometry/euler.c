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
  
 Euler Angles Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <meschach/matrix.h>
#include <meschach/matrix2.h>
#include <math.h>

#include <util.h>

#include "euler.h"
#include "vec3.h"


float normalize_euler_0_2pi(float a)
{
   a = fmod(a, M_PI * 2);
   if (a < 0)
   {
      a += M_PI * 2;
   }
   return a;
}


float normalize_euler_sym_pi(float a)
{
   if (a < -M_PI)
   {
      a = fmod(a, M_PI * 2.0f);
      if (a < -M_PI)
      {
         a += M_PI * 2.0f;
      }
   }
   else if (a > M_PI)
   {
      a = fmod(a, M_PI * 2.0f);
      if (a > M_PI)
      {
         a -= M_PI * 2.0f;
      }
   }
   return a;
}



void euler_normalize(euler_t *euler)
{
   euler->yaw = normalize_euler_0_2pi(euler->yaw);
   euler->pitch = normalize_euler_sym_pi(euler->pitch);
   euler->roll = normalize_euler_sym_pi(euler->roll);
}


struct body_to_neu
{
   VEC *body_acc_vec;
   VEC *world_acc_vec;
   MAT *dcm;
};


void body_to_neu(body_to_neu_t *btn, vec3_t *world, const euler_t *euler, const vec3_t *body)
{
   FOR_N(i, 3)
      btn->body_acc_vec->ve[i] = body->vec[i];

   float theta = euler->pitch;
   float phi = euler->roll;
   float psi = euler->yaw;

   float cos_phi = cos(phi);
   float cos_theta = cos(theta);
   float cos_psi = cos(psi);

   float sin_phi = sin(phi);
   float sin_theta = sin(theta);
   float sin_psi = sin(psi);

   /*
    * Rotation matrix from inertial frame to body frame (for computing expected sensor outputs given yaw, pitch, and roll angles)
    * [ cos(psi) * cos(theta), cos(theta) * sin(psi), -sin(theta) ]
    * [ cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi), cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta), cos(theta) * sin(phi) ]
    * [ sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta), cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi), cos(phi) * cos(theta) ]
    * the transpose of this is used as the inverse DCM below:
    */

   MAT *d = btn->dcm;
   d->me[0][0] = cos_psi * cos_theta;
   d->me[0][1] = cos_psi * sin_phi * sin_theta - cos_phi * sin_psi;
   d->me[0][2] = sin_phi * sin_psi + cos_phi * cos_psi * sin_theta;

   d->me[1][0] = cos_theta * sin_psi;
   d->me[1][1] = cos_phi * cos_psi + sin_phi * sin_psi * sin_theta;
   d->me[1][2] = cos_phi * sin_psi * sin_theta - cos_psi * sin_phi;

   d->me[2][0] = -sin_theta;
   d->me[2][1] = cos_theta * sin_phi;
   d->me[2][2] = cos_phi * cos_theta;

   /*
    * multiply resulting matrix with input vector:
    */
   mv_mlt(d, btn->body_acc_vec, btn->world_acc_vec);

   /*
    * convert meschach vector to output:
    */
   world->vec[0] = btn->world_acc_vec->ve[0];
   world->vec[1] = btn->world_acc_vec->ve[1];
   world->vec[2] = -btn->world_acc_vec->ve[2];
}


body_to_neu_t *body_to_neu_create(void)
{
   body_to_neu_t *btn = (body_to_neu_t *)malloc(sizeof(body_to_neu_t));
   btn->body_acc_vec = v_get(3);
   btn->world_acc_vec = v_get(3);
   btn->dcm = m_get(3, 3);
   return btn;
}

