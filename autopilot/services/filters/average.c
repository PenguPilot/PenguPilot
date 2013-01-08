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
  
 Arithmetic Average Implementation

 Copyright (C) 2010 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "average.h"


void avg_init(avg_data_t *avg_data, int max)
{
   avg_data->avg = 0.0;
   avg_data->count = 0.0;
   avg_data->max_count = max;
   avg_data->sum = 0.0;
}


void avg_add(avg_data_t *avg_data, float value)
{
   /*
    * compute average only if
    * maximum sample count not reached:
    */
   if (avg_data->count < avg_data->max_count)
   {
      avg_data->sum += value;
      avg_data->count++;
      avg_data->avg = avg_data->sum / (float)avg_data->count;
   }
}

