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


#include <util.h>
#include <math.h>

#include "vec2.h"


void vec2_set(vec2_t *vo, float x, float y)
{
   vo->x = x;
   vo->y = y;
}


void vec2_scale(vec2_t *vo, const vec2_t *vi, float factor)
{
   FOR_N(i, 2)
   {
      vo->vec[i] = vi->vec[i] * factor;   
   }
}


float vec2_norm(const vec2_t *v)
{
   return sqrtf(v->x * v->x + v->y * v->y);   
}


void vec2_normalize(vec2_t *vo, const vec2_t *vi)
{
   float norm = vec2_norm(vi);
   if (norm != 0.0f)
   {
      FOR_N(i, 2)
      {
         vo->vec[i] = vi->vec[i] / norm;
      }
   }
}


void vec2_add(vec2_t *vo, const vec2_t *vi1, const vec2_t *vi2)
{
   FOR_N(i, 2)
   {
      vo->vec[i] = vi1->vec[i] + vi2->vec[i];
   }
}


void vec2_sub(vec2_t *vo, const vec2_t *vi1, const vec2_t *vi2)
{
   FOR_N(i, 2)
   {
      vo->vec[i] = vi1->vec[i] - vi2->vec[i];
   }
}


void vec2_ortho_right(vec2_t *vo, const vec2_t *vi)
{
   vo->x = vi->y;
   vo->y = -vi->x;
}


float vec2_inner(const vec2_t *v1, const vec2_t *v2)
{
   return v1->x * v2->x + v1->y * v2->y;
}

 
void vec2_project(vec2_t *vo, const vec2_t *vi1, const vec2_t *vi2)
{
   float scale = vec2_inner(vi1, vi2) / vec2_inner(vi2, vi2);
   vec2_scale(vo, vi2, scale);
}

