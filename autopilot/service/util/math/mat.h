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
  
 Matrix Interface

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __MAT_H__
#define __MAT_H__


#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>

#include "vec.h"


/* generic matrix type */
typedef struct
{
   size_t cols;
   size_t rows;
   real_t *ve;
}
mat_t;


/* declare concrete matrix definition */
#define MAT_DECL(_rows, _cols) \
   typedef struct \
   { \
      size_t cols; \
      size_t rows; \
      real_t *ve; \
      real_t me[_rows][_cols]; \
   } \
   mat##_rows##x##_cols##_t; \
   \
   static void mat##_rows##x##_cols##_init(mat##_rows##x##_cols##_t *mat) \
   { \
      assert(_rows > 0); \
      assert(_cols > 0); \
      mat->rows = _rows; \
      mat->cols = _cols; \
      mat->ve = &mat->me[0][0]; \
      memset(mat->ve, 0, sizeof(real_t) * mat->rows * mat->cols); \
   }


void mat_alloc(mat_t *mat, size_t rows, size_t cols);


#define mat_is_sym(mat_ptr) \
   _mat_is_sym((const mat_t *)mat_ptr)
bool _mat_is_sym(const mat_t *mat);


#define mat_print(mat_ptr) \
   _mat_print((mat_t *)mat_ptr)
void _mat_print(const mat_t *mat);


#define mat_copy(mat_out_ptr, mat_in_ptr) \
   _mat_copy((mat_t *)mat_out_ptr, (const mat_t *)mat_in_ptr)
void _mat_copy(mat_t *out, const mat_t *in);


#define mat_scalar_mul(mat_out_ptr, mat_in_ptr, f) \
   _mat_scalar_mul((mat_t *)mat_out_ptr, (const mat_t *)mat_in_ptr, f)
void _mat_scalar_mul(mat_t *out, const mat_t *in, const real_t f);


#define mat_vec_mul(vec_out_ptr, mat_ptr, vec_in_ptr) \
   _mat_vec_mul((vec_t *)vec_out_ptr, (const mat_t *)mat_ptr, (const vec_t *)vec_in_ptr)
void _mat_vec_mul(vec_t *out, const mat_t *mat, const vec_t *in);


#define mat_add(mat_out_ptr, mat_a_ptr, mat_b_ptr) \
   _mat_add((mat_t *)mat_out_ptr, (const mat_t *)mat_a_ptr, (const mat_t *)mat_b_ptr)
void _mat_add(mat_t *out, const mat_t *a, const mat_t *b);


#define mat_sub(mat_out_ptr, mat_a_ptr, mat_b_ptr) \
   _mat_sub((mat_t *)mat_out_ptr, (const mat_t *)mat_a_ptr, (const mat_t *)mat_b_ptr)
void _mat_sub(mat_t *out, const mat_t *a, const mat_t *b);


#define mat_dim_eq(mat_a_ptr, mat_b_ptr) \
   _mat_dim_eq((const mat_t *)mat_a_ptr, (const mat_t *)mat_b_ptr)
bool _mat_dim_eq(const mat_t *a, const mat_t *b);


#define mat_trans(mat_out_ptr, mat_in_ptr) \
   _mat_trans((mat_t *)mat_out_ptr, (const mat_t *)mat_in_ptr)
void _mat_trans(mat_t *out, const mat_t *in);


#define mat_inv(mat_out_ptr, mat_in_ptr) \
   _mat_inv((mat_t *)mat_out_ptr, (const mat_t *)mat_in_ptr)
void _mat_inv(mat_t *out, const mat_t *in);


#define mat_ident(mat_ptr) \
   _mat_ident((mat_t *)mat_ptr)
void _mat_ident(mat_t *mat);


#define mat_mul(out_mat_ptr, a_mat_ptr, b_mat_ptr) \
   _mat_mul((mat_t *)out_mat_ptr, (const mat_t *)a_mat_ptr, (const mat_t *)b_mat_ptr)
void _mat_mul(mat_t *out, const mat_t *a, const mat_t *b);


#define mmtr_mul(out_mat_ptr, a_mat_ptr, b_mat_ptr) \
   _mmtr_mul((mat_t *)out_mat_ptr, (const mat_t *)a_mat_ptr, (const mat_t *)b_mat_ptr)
void _mmtr_mul(mat_t *out, const mat_t *a, const mat_t *b);



#endif /* __MAT_H__ */

