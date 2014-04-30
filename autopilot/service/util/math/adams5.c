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
 
 Adams4 Integrator Implementation

 Copyright (C) 2013 Alexander Barth, Ilmenau University of Technology
 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <malloc.h>

#include <util.h>

#include "adams5.h"


int adams5_init(adams5_t *a, const size_t dim)
{
   a->dim = dim;
   a->f = (float **)malloc(5 * sizeof(float *));
   FOR_N(i, 5)
      a->f[i] = (float *)malloc((dim + 1) * sizeof(float));
   adams5_reset(a);
   return 1;
}


void adams5_reset(adams5_t *a)
{
   FOR_N(d, a->dim + 1)
      FOR_N(c, 5)
         a->f[c][d] = 0.0;
}


void adams5_term(adams5_t *a)
{
   FOR_N(c, 5)
      free(a->f[c]);
   free(a->f);
}


void adams5_run(adams5_t *a, float *out, float *in, float ts, int enabled)
{ 
   /* rotate ring buffer */ 
   float *temp = a->f[4];
   FOR_N(c, 4)  
      a->f[4 - c] = a->f[3 - c];
   a->f[0] = temp;
 
   if (enabled)
   {
      /* set f0 and run adams5 integrator: */
      for (size_t d = 0; d < a->dim; d++)
      {
         a->f[0][d] = in[d];
         a->f[0][a->dim] = ts;

         const float coeff[5] = { 1901.0f / 720.0f,
                                 -1387.f / 360.0f,
                                  109.0f / 30.0f,
                                 -637.0f / 360.0f,
                                  251.0f / 720.0f};
         FOR_N(c, 5)
            out[d] += coeff[c] * a->f[c][d] * a->f[d][a->dim] /* dt */;
      }
   }
}

