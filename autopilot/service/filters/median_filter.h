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
 * median_filter.h
 *
 *  Created on: 18.06.2010
 *      Author: tobi
 */

#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H


typedef struct
{
   float *history;
   float *sort;
   size_t size;
   size_t count;
}
median_filter_t;


void median_filter_init(median_filter_t *filter, size_t size);

int median_filter_ready(median_filter_t *filter);

float median_filter_run(median_filter_t *filter, float val);


#endif /* MEDIAN_FILTER_H */
