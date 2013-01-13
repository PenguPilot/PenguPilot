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
   interval computation implementation

   Copyright (C) 2012 Tobias Simon

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
*/


#include <stdint.h>
#include <time.h>

#include "interval.h"


void interval_init(interval_t *interval)
{
   clock_gettime(CLOCK_MONOTONIC, &interval->prev);
}


static int64_t ts_diff(struct timespec *timeA_p, struct timespec *timeB_p)
{
   return ((timeA_p->tv_sec * 1000000000) + timeA_p->tv_nsec) -
          ((timeB_p->tv_sec * 1000000000) + timeB_p->tv_nsec);
}


float interval_measure(interval_t *interval)
{
   return interval_measure_ms(interval) / 1000.0f;
}


float interval_measure_ms(interval_t *interval)
{
   clock_gettime(CLOCK_MONOTONIC, &interval->curr);
   double dt = (double)ts_diff(&interval->curr, &interval->prev) / 1000000.0f;
   interval->prev = interval->curr;
   return dt;
}

