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
  
 Generic Vectors Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include "vec.h"

#include <util.h>

#include <stdio.h>
#include <assert.h>
#include <malloc.h>
#include <string.h>


void _vec_copy(vec_t *out, const vec_t *in)
{
   assert(out && in);
   assert(out->dim == in->dim);
   memcpy((void *)out->ve, (const void *)in->ve, sizeof(real_t) * out->dim);
}


bool _vec_equal(const vec_t *a, const vec_t *b)
{
   assert(a && b);
   assert(a->dim == b->dim);
   return memcmp((const void *)a->ve, (const void *)b->ve, sizeof(real_t) * a->dim) == 0;
}


void vec_alloc(vec_t *vec, size_t dim)
{
   assert(dim > 0);

   vec->dim = dim;
   vec->ve = (real_t *)malloc(sizeof(real_t) * vec->dim);
   memset(vec->ve, 0, sizeof(real_t) * vec->dim);
}


void _vec_fill(vec_t *vec, const real_t *values)
{
   assert(vec && values);

   memcpy((void *)&vec->ve[0], (void *)values, vec->dim * sizeof(real_t));
}


real_t _vec_norm(const vec_t *vec)
{
   assert(vec);

   real_t sum = REAL(0);
   FOR_N(i, vec->dim)
      sum += vec->ve[i] * vec->ve[i];
   return real_sqrt(sum);
}


void _vec_scale(vec_t *vec, const real_t f)
{
   assert(vec);

   FOR_N(i, vec->dim)
      vec->ve[i] *= f;
}


void _vec_scalar_mul(vec_t *out, const vec_t *in, const real_t f)
{
   assert(out && in);
   assert(out->dim == in->dim);

   FOR_N(i, out->dim)
      out->ve[i] = in->ve[i] * f;
}


void _vec_print(const vec_t *vec)
{
   assert(vec);

   printf("[");
   FOR_N(i, vec->dim)
   {
      printf("%f", vec->ve[i]);
      if (i < vec->dim - 1)
         printf(", ");
   }
   printf("]^T \n");
}


void _vec_add(vec_t *out, const vec_t *a, const vec_t *b)
{
   assert(out && a && b);
   assert(out->dim == a->dim);
   assert(out->dim == b->dim);

   FOR_N(i, out->dim)
      out->ve[i] = a->ve[i] + b->ve[i];
}


void _vec_sub(vec_t *out, const vec_t *a, const vec_t *b)
{
   assert(out && a && b);
   assert(out->dim == a->dim);
   assert(out->dim == b->dim);

   FOR_N(i, out->dim)
      out->ve[i] = a->ve[i] - b->ve[i];
}

