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
  
 Generic Vectors Interface

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#ifndef __VEC_H__
#define __VEC_H__


#include "real.h"

#include <stdlib.h>
#include <assert.h>
#include <stdbool.h>
#include <string.h>


/* generic vector type */
typedef struct
{
   size_t dim;
   real_t *ve;
}
vec_t;


void vec_alloc(vec_t *vec, size_t dim);


/* declare concrete vector definition */
#define VEC_DECL(_dim) \
   typedef struct \
   { \
      size_t dim; \
      real_t *ve; \
      union { \
         real_t data[_dim]; \
         struct { \
            real_t x; \
            real_t y; \
            real_t z; \
         }; \
         struct { \
            real_t n; \
            real_t e; \
            real_t u; \
         }; \
      }; \
   } \
   vec##_dim##_t; \
   \
   static void vec##_dim##_init(vec##_dim##_t *vec) \
   { \
      assert(_dim > 0); \
      vec->dim = _dim; \
      vec->ve = (real_t *)&vec->data; \
      memset(vec->ve, 0, sizeof(real_t) * vec->dim); \
   }


#define vec_copy(vec_out, vec_in) \
   _vec_copy((vec_t *)vec_out, (const vec_t *)vec_in)
void _vec_copy(vec_t *out, const vec_t *in);


#define vec_equal(vec_a, vec_b) \
   _vec_equal((const vec_t *)vec_a, (const vec_t *)vec_b)
bool _vec_equal(const vec_t *a, const vec_t *b);


#define vec_fill(vec_ptr, values) \
   _vec_fill((vec_t *)vec_ptr, values)
void _vec_fill(vec_t *vec, const real_t *values);


#define vec_norm(vec_ptr) \
   _vec_norm((const vec_t *)vec_ptr)
real_t _vec_norm(const vec_t *vec);


#define vec_print(vec_ptr) \
   _vec_print((const vec_t *)vec_ptr)
void _vec_print(const vec_t *vec);


#define vec_scale(vec_ptr, f) \
   _vec_scale((vec_t *)vec_ptr, f)
void _vec_scale(vec_t *vec, const real_t f);


#define vec_scalar_mul(vec_out, vec_in, f) \
   _vec_scalar_mul((vec_t *)vec_out, (const vec_t *)vec_in, f)
void _vec_scalar_mul(vec_t *out, const vec_t *in, const real_t f);


#define vec_add(vec_out_ptr, vec_a_ptr, vec_b_ptr) \
   _vec_add((vec_t *)vec_out_ptr, (const vec_t *)vec_a_ptr, (const vec_t *)vec_b_ptr)
void _vec_add(vec_t *out, const vec_t *a, const vec_t *b);


#define vec_sub(vec_out_ptr, vec_a_ptr, vec_b_ptr) \
   _vec_sub((vec_t *)vec_out_ptr, (const vec_t *)vec_a_ptr, (const vec_t *)vec_b_ptr)
void _vec_sub(vec_t *out, const vec_t *a, const vec_t *b);


#define vec_normalize(vec_ptr) \
   _vec_normalize((vec_t *)vec_ptr)
void _vec_normalize(vec_t *vec);


#endif /* __VEC_H__ */

