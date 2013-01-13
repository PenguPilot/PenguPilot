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
  
 2D Vector Interface

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#ifndef __VEC2_H__
#define __VEC2_H__


typedef union
{
   struct
   {
      float x;
      float y;
   };
   float vec[2];
}
vec2_t;


void vec2_set(vec2_t *vo, float x, float y);

void vec2_scale(vec2_t *vo, const vec2_t *vi, float factor);

float vec2_norm(const vec2_t *v);

void vec2_normalize(vec2_t *vo, const vec2_t *vi);

void vec2_add(vec2_t *vo, const vec2_t *vi1, const vec2_t *vi2);

void vec2_sub(vec2_t *vo, const vec2_t *vi1, const vec2_t *vi2);

void vec2_ortho_right(vec2_t *vo, const vec2_t *vi);

float vec2_inner(const vec2_t *v1, const vec2_t *v2);
 
void vec2_project(vec2_t *vo, const vec2_t *vi1, const vec2_t *vi2);


#endif /* __VEC2_H__ */

