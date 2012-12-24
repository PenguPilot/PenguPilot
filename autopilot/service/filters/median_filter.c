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
 * median_filter.c
 *
 *  Created on: 18.06.2010
 *      Author: tobi
 */


#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#include "median_filter.h"
#include "util.h"



void median_filter_init(median_filter_t *filter, size_t size)
{
   filter->history = malloc(size * sizeof(float));
   memset(filter->history, 0, size * sizeof(float));
   filter->sort = malloc(size * sizeof(float));
   memset(filter->sort, 0, size * sizeof(float));
   filter->size = size;
   filter->count = 0;
}


int median_filter_ready(median_filter_t *filter)
{
   return filter->count == filter->size;
}


float median_filter_run(median_filter_t *filter, float val)
{
   unsigned int i;
   for (i = 0; i < filter->size - 1; i++)
   {
      filter->history[i] = filter->history[i + 1];
   }
   filter->history[i] = val;
   if (filter->count < filter->size)
   {
      filter->count++;
   }
   memcpy(filter->sort, filter->history, sizeof(float) * filter->size);
   qsort(filter->sort, filter->size, sizeof(float), compare_floats);
   return filter->sort[filter->size / 2 + ((filter->size & 1) ? 1 : 0)];
}
