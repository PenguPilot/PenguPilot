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
  
 Conversions Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include <math.h>


float deg2rad(float x)
{
   return x * (float)(M_PI / 180.0);
}


float rad2deg(float x)
{
   return x * (float)(180.0 / M_PI);
}


float norm_angle_0_2pi(float a)
{
   a = fmod(a, M_PI * 2);
   if (a < 0)
   {
      a += M_PI * 2;
   }
   return a;
}


float norm_angle_sym_pi(float a)
{
   if (a < -M_PI)
   {
      a = fmod(a, M_PI * 2.0f);
      if (a < -M_PI)
      {
         a += M_PI * 2.0f;
      }
   }
   else if (a > M_PI)
   {
      a = fmod(a, M_PI * 2.0f);
      if (a > M_PI)
      {
         a -= M_PI * 2.0f;
      }
   }
   return a;
}

