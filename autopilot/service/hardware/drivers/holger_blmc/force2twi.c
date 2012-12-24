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


#include <math.h>

#include <util.h>

#include "force2twi.h"
#include "holger_blmc.h"
#include "../../../control/basic/control_param.h"


/* rpm ^ 2 = a * voltage ^ 1.5 * i2c ^ b */
#define CTRL_F_A 609.6137f
#define CTRL_F_B 1.3154f


int force2twi_calc(uint8_t *i2c, const float voltage, const float *rpm_square, const size_t n_motors)
{
   int int_enable = 1;
   /* computation i2c values out of rpm_square by the inverse of: rpm ^ 2 = a * voltage ^ 1.5 * i2c ^ b */
   FOR_N(i, n_motors)
   {
      float temp = powf((rpm_square[i] / CTRL_F_A * powf(voltage, -1.5f)), 1.0f / CTRL_F_B);
      if (temp > (float)HOLGER_I2C_MIN)
      {   
         if (temp > (float)HOLGER_I2C_MAX)
         {   
            i2c[i] = (uint8_t)HOLGER_I2C_MAX;
            int_enable = 0;
         }
         else
         {   
            i2c[i] = (uint8_t)temp;
         }
      }
      else
      {   
         i2c[i] = (uint8_t)HOLGER_I2C_MIN;
         int_enable = 0;
      }
   }
   return int_enable;
}

