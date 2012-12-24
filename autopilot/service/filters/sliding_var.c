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


/*
   Sliding Variance

   Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
 */


#include <util.h>

#include "sliding_var.h"


void sliding_var_init(sliding_var_t *sliding_var, size_t wnd_size, float init)
{
   sliding_avg_init(&sliding_var->avg, wnd_size, 0.0);
   sliding_var->wnd_size = wnd_size + 1;
   sliding_var->hist = malloc(sizeof(float) * sliding_var->wnd_size);
   ASSERT_NOT_NULL(sliding_var->hist);
   sliding_var->square_sum = init * wnd_size;
   FOR_N(i, sliding_var->wnd_size)
   {
      sliding_var->hist[i] = init;
   }
   sliding_var->pos = 0;
}


float sliding_var_calc(sliding_var_t *sliding_var, float val)
{
   float avg = sliding_avg_calc(&sliding_var->avg, val);
   int next_pos = (sliding_var->pos + 1) % sliding_var->wnd_size;
   float last = sliding_var->hist[next_pos];
   float square_diff = (val - avg) * (val - avg);
   sliding_var->square_sum = sliding_var->square_sum - last + square_diff;
   sliding_var->hist[sliding_var->pos] = square_diff;
   sliding_var->pos = next_pos;
   return sliding_var->square_sum / (sliding_var->wnd_size - 2);
}

