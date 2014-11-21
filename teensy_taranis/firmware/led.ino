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
 
 Blinking LED Implementation

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include "led.h"


static const int led = 13;


void led_init()
{
   pinMode(led, OUTPUT);
}


int led_update(int flags)
{
   static int val = 0;
   static int state = 0;
   static int k = 0;
   static const int steps = 200000;
   state++;
   if (state < steps)
   {
      if ((k++ >= 4000) && (flags & S_PORT_ACTIVITY))
      {
         k = 0;
         val ^= 1;
      }
   }
   else if (state == steps)
   {
      val = 0;
   }
   else if (state > steps && state < 2 * steps)
   {
      if ((k++ >= 8000) && (flags & S_BUS_ACTIVITY))
      {
         k = 0;
         val ^= 1;
      }
   }
   else if (state == 2 * steps)
   {
      val = 0;
   }
   else if (state > 2 * steps && state < 3 * steps)
   {
      if ((k++ >= 16000) && (flags & AP_ACTIVITY))
      {
         k = 0;
         val ^= 1;
      }
   }
   else if (state > 3 * steps)
   {
      state = 0;
      flags = 0;
   }

   digitalWrite(led, val);
   return flags;
}
