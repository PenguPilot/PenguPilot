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
  
 Force to Motor Setpoint Converter Implementation

 Copyright (C) 2012 Benjamin Jahn, Ilmenau University of Technology
 Copyright (C) 2012 Alexander Barth, Ilmenau University of Technology
 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "force_to_setpoint.h"

#include <math.h>
#include <util.h>


static int force_to_setpoint(force_to_setpoint_t *ftos, float *setpoint, float voltage, float force)
{
   int int_enable;
   float sp = powf((force / ftos->a * powf(voltage, -1.5f)), 1.0f / ftos->b);
   if (sp > ftos->sp_max)
   {   
      sp = ftos->sp_max;
      int_enable = 0;
   }
   else if (sp < ftos->sp_min)
   {
      sp = ftos->sp_min;
      int_enable = 0;
   }
   else
   {
      int_enable = 1;   
   }
   *setpoint = sp;
   return int_enable;
}


int forces_to_setpoints(force_to_setpoint_t *ftos, float *setpoints, float voltage, float *forces, int n_motors)
{
   int int_enable = 1;
   FOR_N(i, n_motors)
   {
      if (force_to_setpoint(ftos, &setpoints[i], voltage, forces[i]) == 0)
      {
         int_enable = 0;   
      }
   }
   return int_enable;
}

