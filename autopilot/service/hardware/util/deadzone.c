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
  
 Deadzone Implementation

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <math.h>

#include "deadzone.h"



void deadzone_init(deadzone_t *dz, float xmin, float xmax, float ymax)
{
   dz->x0 = xmin;
   dz->ymax = ymax;
   dz->a = ymax / (xmax - xmin);
}


float deadzone_calc(deadzone_t *dz, float x)
{
   float y;
   float sign = (x >= 0) ? 1.0f : -1.0f;
   x = fabs(x);
   if (x <= dz->x0)
   {
      y = 0.0f;
   }
   else
   {
      y = fmin(dz->ymax, dz->a * (x - dz->x0));
   }
   return y * sign;
}

