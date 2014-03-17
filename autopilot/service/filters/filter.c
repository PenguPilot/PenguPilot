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
  
 Filter Library Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology
 Copyright (C) 2013 Alexander Barth, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <math.h>
#include <malloc.h>

#include <util.h>

#include "filter.h"


static float filter_constant(float fg)
{
   return 1.0f / (2.0f * M_PI * fg);
}


void filter1_lp_init(Filter1 *filter, float fg, float Ts, int signal_dim)
{
   /* calculate filter time constant */
   float T = filter_constant(fg);

   /* filter coefficients for 2nd order highpass filter */
   float a[1];
   a[0] = (2.0f * Ts) / (2.0f * T + Ts) - 1.0f;

   float b[2];
   b[0] = Ts / (2.0f * T + Ts);
   b[1] = b[0];
   filter1_init(filter, a, b, Ts, signal_dim);
}


void filter1_hp_init(Filter1 *filter, float fg, float Ts, int signal_dim)
{
   /* calculate filter time constant */
   float T = filter_constant(fg);

   /* filter coefficients for 2nd order highpass filter */
   float a[1];
   a[0] = (2.0f * Ts) / (2.0f * T + Ts) - 1.0f;

   float b[2];
   b[0] = 2.0f / (2.0f * T + Ts);
   b[1] = -b[0];
   filter1_init(filter, a, b, Ts, signal_dim);
}


void filter1_init(Filter1 *filter, float *a, float *b, float Ts, int signal_dim)
{
   filter->Ts = Ts;
   filter->signal_dim = signal_dim;
    
   filter->z = malloc(signal_dim * sizeof(float));
   ASSERT_NOT_NULL(filter->z);

   FOR_N(i, signal_dim)
   {
      filter->z[i] = 0.0f;
   }
   filter->Ts = Ts;
   filter->a1 = a[0];
    
   filter->b0 = b[0];
   filter->b1 = b[1];
}


void filter1_run(Filter1 *filter, float *u_in, float *y)
{
   FOR_N(i, filter->signal_dim)
   {
      float u = u_in[i];
      y[i]         = filter->b0 * u + filter->z[i];
      filter->z[i] = filter->b1 * u - filter->a1 * y[i];
   }
}


void filter2_lp_init(Filter2 *filter, float fg, float d, float Ts, int signal_dim)
{
   /* calculate filter fime constant */
   float T = filter_constant(fg);

   /* filter coefficients for 2nd order highpass filter */
   float a[2];
   a[0] = (2.0f * Ts * Ts - 8.0f * T * T) / (4.0f * T * T + 4.0f * d * T * Ts + Ts * Ts);
   a[1] = (4.0f * T * T - 4.0f * d * T * Ts + Ts * Ts) / (4.0f * T * T + 4.0f * d * T * Ts + Ts * Ts);

   float b[3];
   b[0] = Ts * Ts / (4.0f * T * T + 4.0f * d * T * Ts + Ts * Ts);
   b[1] = 2.0f * b[0];
   b[2] = b[0];
   filter2_init(filter, a, b, Ts, signal_dim);
}


void filter2_hp_init(Filter2 *filter, float fg, float d, float Ts, int signal_dim)
{
   /* calculate filter fime constant */
   float T = filter_constant(fg);

   /* Filter coefficients for 2nd order highpass filter */
   float a[2];
   a[0] = (2.0f * Ts * Ts - 8.0f * T * T) / (4.0f * T * T + 4.0f * d * T * Ts + Ts * Ts);
   a[1] = (4.0f * T * T - 4.0f * d * T * Ts + Ts * Ts) / (4.0f * T * T + 4.0f * d * T * Ts + Ts * Ts);

   float b[3];
   b[0] = (2.0f * Ts) / (4.0f * T * T + 4.0f * d * T * Ts + Ts * Ts);
   b[1] = 0.0f;
   b[2] = -b[0];
   filter2_init(filter, a, b, Ts, signal_dim);
}


void filter2_init(Filter2 *filter, float *a, float *b, float Ts, int signal_dim)
{
   filter->Ts = Ts;
   filter->signal_dim = signal_dim;
    
   filter->z1 = malloc(signal_dim * sizeof(float)); 
   ASSERT_NOT_NULL(filter->z1);
   filter->z2 = malloc(signal_dim * sizeof(float));
   ASSERT_NOT_NULL(filter->z2);

   FOR_N(i, signal_dim)
   {
      filter->z1[i] = 0.0f;
      filter->z2[i] = 0.0f;
   }
   filter->Ts = Ts;
   filter->a1 = a[0];
   filter->a2 = a[1];
    
   filter->b0 = b[0];
   filter->b1 = b[1];
   filter->b2 = b[2];
}


void filter2_run(Filter2 *filter, float *u_in, float *y)
{
   FOR_N(i, filter->signal_dim)
   {
      float u = u_in[i];
      y[i]          = filter->b0 * u                     + filter->z1[i];
      filter->z1[i] = filter->b1 * u - filter->a1 * y[i] + filter->z2[i];
      filter->z2[i] = filter->b2 * u - filter->a2 * y[i];
   }
}

