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



#include <malloc.h>

#include "adams4.h"


int adams4_init(adams4_t *a, const size_t dim)
{
   a->dim = dim;
   a->f0 = malloc(dim * sizeof(float));
   a->f1 = malloc(dim * sizeof(float));
   a->f2 = malloc(dim * sizeof(float));
   a->f3 = malloc(dim * sizeof(float));

   adams4_reset(a);
   return 1;
}


void adams4_reset(adams4_t *a)
{
   for (size_t i = 0; i < a->dim; i++)
   {  
      a->f0[i] = 0.0;
      a->f1[i] = 0.0;
      a->f2[i] = 0.0;
      a->f3[i] = 0.0;
   }
}


void adams4_term(adams4_t *a)
{
   free(a->f0);
   free(a->f1);
   free(a->f2);
   free(a->f3);
}


/*
 * x_(k+1) = x_(k) + Ts*(55 * x_(k) - 59 * x_(k-1) + 37 * x_(k-2) - 9 * x_(k-3))/24
*/
void adams4_run(adams4_t *a, float *x, float ts, int enabled)
{
   if (enabled)
   {
      /* run adams4 algorithm: */
      for (size_t i = 0; i < a->dim; i++)
      {
         x[i] += ts * (55.0f * a->f0[i] - 59.0f * a->f1[i] + 37.0f * a->f2[i] - 9.0f * a->f3[i]) / 24.0f;
      }
   }
   /* rotate ring buffer; TS: is it correct not to include this in (enabled)? */
   float *temp = a->f3;
   a->f3 = a->f2;
   a->f2 = a->f1;
   a->f1 = a->f0;
   a->f0 = temp;
}

