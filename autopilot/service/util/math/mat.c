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
  
 Matrix Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "mat.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <malloc.h>
#include <util.h>


void mat_alloc(mat_t *mat, size_t rows, size_t cols)
{
   assert(rows > 0);
   assert(cols > 0);
   
   mat->rows = rows;
   mat->cols = cols;
   mat->ve = (real_t *)malloc(sizeof(real_t) * mat->rows * mat->cols);
   assert(mat->ve);
   memset(mat->ve, 0, sizeof(real_t) * mat->rows * mat->cols);
}


#define ME(mat_ptr, i, j) \
   mat_ptr->ve[(i) * mat_ptr->cols + (j)]


bool _mat_is_sym(const mat_t *mat)
{
   assert(mat);

   return mat->cols == mat->rows;
}


void _mat_vec_mul(vec_t *out, const mat_t *mat, const vec_t *in)
{
   assert(out && mat && in);
   assert(mat->rows == out->dim);
   assert(mat->cols == in->dim);

   FOR_N(i, mat->rows)
   {
      real_t sum = REAL(0);
      FOR_N(j, mat->cols)
         sum += ME(mat, i, j) * in->ve[j];
      out->ve[i] = sum;
   }
}


void _mat_print(const mat_t *mat)
{
   assert(mat);

   FOR_N(i, mat->rows)
   {
      FOR_N(j, mat->cols)
      {
         printf("%f", ME(mat, i, j));
         if (j < mat->cols - 1)
            printf(", ");
      }
      printf("\n");
   }
}


void _mat_trans(mat_t *out, const mat_t *in)
{
   assert(out && in && out != in);
   assert(out->rows == in->cols);
   assert(out->cols == in->rows);

   FOR_N(i, in->rows)
      FOR_N(j, in->cols)
         ME(out, j, i) = ME(in, i, j);
}


void _mat_inv(mat_t *out, const mat_t *in)
{
   assert(out && in);
   assert(mat_is_sym(in));

   if (in->rows == 1)
   {
      out->ve[0] = REAL(1) / in->ve[0];   
   }
   else if (in->rows == 2)
   {
      const real_t inv_det = REAL(1) / (in->ve[0] * in->ve[3] - in->ve[2] * in->ve[1]);

      out->ve[0] =  in->ve[3] * inv_det;
      out->ve[1] = -in->ve[1] * inv_det;
      out->ve[2] = -in->ve[2] * inv_det;
      out->ve[3] =  in->ve[0] * inv_det;
   }
   else if (in->rows == 3)
   {
      const real_t inv_det = REAL(1) / 
          (ME(in, 0, 0) * (ME(in, 1, 1) * ME(in, 2, 2) - ME(in, 2, 1) * ME(in, 1, 2))
         - ME(in, 0, 1) * (ME(in, 1, 0) * ME(in, 2, 2) - ME(in, 1, 2) * ME(in, 2, 0))
         + ME(in, 0, 2) * (ME(in, 1, 0) * ME(in, 2, 1) - ME(in, 1, 1) * ME(in, 2, 0)));

      ME(out, 0, 0) =  (ME(in, 1, 1) * ME(in, 2, 2) - ME(in, 2, 1) * ME(in, 1, 2)) * inv_det;
      ME(out, 0, 1) = -(ME(in, 0, 1) * ME(in, 2, 2) - ME(in, 0, 2) * ME(in, 2, 1)) * inv_det;
      ME(out, 0, 2) =  (ME(in, 0, 1) * ME(in, 1, 2) - ME(in, 0, 2) * ME(in, 1, 1)) * inv_det;
      ME(out, 1, 0) = -(ME(in, 1, 0) * ME(in, 2, 2) - ME(in, 1, 2) * ME(in, 2, 0)) * inv_det;
      ME(out, 1, 1) =  (ME(in, 0, 0) * ME(in, 2, 2) - ME(in, 0, 2) * ME(in, 2, 0)) * inv_det;
      ME(out, 1, 2) = -(ME(in, 0, 0) * ME(in, 1, 2) - ME(in, 1, 0) * ME(in, 0, 2)) * inv_det;
      ME(out, 2, 0) =  (ME(in, 1, 0) * ME(in, 2, 1) - ME(in, 2, 0) * ME(in, 1, 1)) * inv_det;
      ME(out, 2, 1) = -(ME(in, 0, 0) * ME(in, 2, 1) - ME(in, 2, 0) * ME(in, 0, 1)) * inv_det;
      ME(out, 2, 2) =  (ME(in, 0, 0) * ME(in, 1, 1) - ME(in, 1, 0) * ME(in, 0, 1)) * inv_det;
   }
   else
   {
      assert(false);   
   }
}


bool _mat_dim_eq(const mat_t *a, const mat_t *b)
{
   assert(a && b);

   return a->rows == b->rows && a->cols == b->cols;   
}


void _mat_scalar_mul(mat_t *out, const mat_t *in, const real_t f)
{
   assert(out && in && out != in);
   assert(mat_dim_eq(out, in));

   FOR_N(i, in->rows)
      FOR_N(j, in->cols)
         ME(out, i, j) = ME(in, i, j) * f;
}


void _mat_copy(mat_t *out, const mat_t *in)
{
   assert(out && in);
   assert(mat_dim_eq(out, in));

   memcpy(out->ve, in->ve, sizeof(real_t) * out->rows * out->cols);
}


void _mat_add(mat_t *out, const mat_t *a, const mat_t *b)
{
   assert(out && a && b);
   assert(mat_dim_eq(out, a));
   assert(mat_dim_eq(out, b));

   FOR_N(i, out->rows)
      FOR_N(j, out->cols)
         ME(out, i, j) = ME(a, i, j) + ME(b, i, j);
}


void _mat_sub(mat_t *out, const mat_t *a, const mat_t *b)
{
   assert(out && a && b);
   assert(mat_dim_eq(out, a));
   assert(mat_dim_eq(out, b));

   FOR_N(i, out->rows)
      FOR_N(j, out->cols)
         ME(out, i, j) = ME(a, i, j) - ME(b, i, j);
}


void _mat_ident(mat_t *mat)
{
   assert(mat);
   assert(mat_is_sym(mat));

   FOR_N(i, mat->rows)
      FOR_N(j, mat->cols)
         ME(mat, j, i) = (i == j) * REAL(1);
}


void _mat_mul(mat_t *out, const mat_t *a, const mat_t *b)
{
   assert(out && a && b);
   assert(a->cols == b->rows);
   assert(a->rows == out->rows);
   assert(b->cols == out->cols);
   
   FOR_N(i, out->rows)
      FOR_N(j, out->cols)
      {
         real_t sum = REAL(0);
         FOR_N(k, a->cols)
            sum += (ME(a, i, k)) * (ME(b, k, j));
         ME(out, i, j) = sum;
      }
}


void _mmtr_mul(mat_t *out, const mat_t *a, const mat_t *b)
{
   assert(out && a && b);
   assert(a->cols == b->rows);
   assert(a->rows == out->rows);
   assert(b->cols == out->cols);
   
   FOR_N(i, out->rows)
      FOR_N(j, out->cols)
      {
         real_t sum = REAL(0);
         FOR_N(k, a->cols)
            sum += (ME(a, i, k)) * (ME(b, j, k));
         ME(out, i, j) = sum;
      }
}

