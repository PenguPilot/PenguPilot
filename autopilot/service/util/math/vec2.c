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
  
 2D Vectors Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include "vec2.h"

#include <util.h>
#include <string.h>


void vec2_init(vec2_t *vec)
{
   assert(vec);
   vec->dim = 2;
   vec->ve = &vec->data[0];
   memset(vec->ve, 0, sizeof(real_t) * vec->dim);
}


void vec2_set(vec2_t *vec, real_t x, real_t y)
{
   assert(vec);
   vec->dim = 2;
   vec->ve = &vec->data[0];
   vec->x = x;
   vec->y = y;
}


void vec2_scale(vec2_t *vo, const vec2_t *vi, float factor)
{
   assert(vo && vi);
   assert(vo->dim == 2);
   assert(vi->dim == 2);

   FOR_N(i, 2)
   {
      vo->ve[i] = vi->ve[i] * factor;   
   }
}


float vec2_norm(const vec2_t *v)
{
   assert(v);
   assert(v->dim == 2);

   return sqrtf(v->x * v->x + v->y * v->y);   
}


void vec2_normalize(vec2_t *vo, const vec2_t *vi)
{
   assert(vo);
   assert(vo->dim == 2);
   
   float norm = vec2_norm(vi);
   if (norm != 0.0f)
   {
      FOR_N(i, 2)
      {
         vo->ve[i] = vi->ve[i] / norm;
      }
   }
}


void vec2_add(vec2_t *vo, const vec2_t *vi1, const vec2_t *vi2)
{
   assert(vo && vi1 && vi2);
   assert(vo->dim == 2 && vi1->dim == 2 && vi2->dim == 2);

   FOR_N(i, 2)
   {
      vo->ve[i] = vi1->ve[i] + vi2->ve[i];
   }
}


void vec2_sub(vec2_t *vo, const vec2_t *vi1, const vec2_t *vi2)
{
   assert(vo && vi1 && vi2);
   assert(vo->dim == 2 && vi1->dim == 2 && vi2->dim == 2);

   FOR_N(i, 2)
   {
      vo->ve[i] = vi1->ve[i] - vi2->ve[i];
   }
}


void vec2_ortho_right(vec2_t *vo, const vec2_t *vi)
{   
   assert(vo && vi);
   assert(vo->dim == 2 && vi->dim == 2);

   vo->x = vi->y;
   vo->y = -vi->x;
}


float vec2_inner(const vec2_t *v1, const vec2_t *v2)
{   
   assert(v1 && v2);
   assert(v1->dim == 2 && v2->dim == 2);

   return v1->x * v2->x + v1->y * v2->y;
}

 
void vec2_project(vec2_t *vo, const vec2_t *vi1, const vec2_t *vi2)
{   
   assert(vo && vi1 && vi2);
   assert(vo->dim == 2 && vi1->dim == 2 && vi2->dim == 2);

   float scale = vec2_inner(vi1, vi2) / vec2_inner(vi2, vi2);
   vec2_scale(vo, vi2, scale);
}


void vec2_rotate(vec2_t *vo, const vec2_t *vi, float angle)
{
   assert(vo && vi);
   assert(vo != vi);
   assert(vo->dim == 2 && vi->dim == 2);

   float sa = real_sin(angle);
   float ca = real_cos(angle);
   vo->x = vi->x * ca - vi->y * sa;
   vo->y = vi->x * sa + vi->y * ca;
}

