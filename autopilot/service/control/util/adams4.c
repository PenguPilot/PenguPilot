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

#include "adams4.h"


int adams4_init(adams4_t *a, const size_t dim)
{
   a->dim = dim;
   a->f0 = malloc((dim + 1) * sizeof(float));
   a->f1 = malloc((dim + 1) * sizeof(float));
   a->f2 = malloc((dim + 1) * sizeof(float));
   a->f3 = malloc((dim + 1) * sizeof(float));
   a->f4 = malloc((dim + 1) * sizeof(float));

   adams4_reset(a);
   return 1;
}


void adams4_reset(adams4_t *a)
{
   for (size_t i = 0; i < a->dim + 1; i++)
   {  
      a->f0[i] = 0.0;
      a->f1[i] = 0.0;
      a->f2[i] = 0.0;
      a->f3[i] = 0.0;
      a->f4[i] = 0.0;
   }
}


void adams4_term(adams4_t *a)
{
   free(a->f0);
   free(a->f1);
   free(a->f2);
   free(a->f3);
   free(a->f4);
}


/*
 * x_(k+1) = x_(k) + Ts*(55 * x_(k) - 59 * x_(k-1) + 37 * x_(k-2) - 9 * x_(k-3))/24
*/
void adams4_run(adams4_t *a, float *out, float *in, float ts, int enabled)
{
   if (enabled)
   {
      /* set f0 and run adams4 integrator: */
      for (size_t i = 0; i < a->dim; i++)
      {
         a->f0[i] = in[i];
         a->f0[a->dim] = ts;

         const float coeff[5] = { 191.0f / 720.0f,
                                 -1387.f / 360.0f,
                                  109.0f / 30.0f,
                                 -637.0f / 360.0f,
                                  251.0f / 720.0f};

         out[i] += coeff[0] * a->f0[i] * a->f0[a->dim];
         out[i] += coeff[1] * a->f1[i] * a->f1[a->dim];
         out[i] += coeff[2] * a->f2[i] * a->f2[a->dim];
         out[i] += coeff[3] * a->f3[i] * a->f3[a->dim];
         out[i] += coeff[4] * a->f4[i] * a->f4[a->dim];
      }
   }
   /* rotate ring buffer; TS: is it correct not to include this in (enabled)? */
   float *temp = a->f4;
   a->f4 = a->f3;
   a->f3 = a->f2;
   a->f2 = a->f1;
   a->f1 = a->f0;
   a->f0 = temp;
}

