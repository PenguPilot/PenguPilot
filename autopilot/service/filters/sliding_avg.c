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
   Sliding Average

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

#include "sliding_avg.h"


void sliding_avg_init(sliding_avg_t *sliding_avg, size_t wnd_size, float init)
{
   sliding_avg->wnd_size = wnd_size + 1;
   sliding_avg->hist = malloc(sizeof(float) * sliding_avg->wnd_size);
   ASSERT_NOT_NULL(sliding_avg->hist);
   sliding_avg->sum = init * wnd_size;
   FOR_N(i, sliding_avg->wnd_size)
   {
      sliding_avg->hist[i] = init;
   }
   sliding_avg->pos = 0;
}


float sliding_avg_calc(sliding_avg_t *sliding_avg, float val)
{
   int next_pos = (sliding_avg->pos + 1) % sliding_avg->wnd_size;
   float last = sliding_avg->hist[next_pos];
   sliding_avg->sum = sliding_avg->sum - last + val;
   sliding_avg->hist[sliding_avg->pos] = val;
   sliding_avg->pos = next_pos;
   return sliding_avg->sum / (sliding_avg->wnd_size - 1);
}

