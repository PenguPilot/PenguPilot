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
  
 Arduino Power Parser Implementation

 Copyright (C) 2014 Jan Roemisch, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <assert.h>

#include "power_common.h"
#include "checksum.h"


static enum
{
   READ_SYNC0,
   READ_SYNC1,
   READ_VOLTAGE0,
   READ_VOLTAGE1,
   READ_VOLTAGE2,
   READ_VOLTAGE3,
   READ_CURRENT0,
   READ_CURRENT1,
   READ_CURRENT2,
   READ_CURRENT3,
   READ_CS0,
   READ_CS1
}
state = READ_SYNC0;

static uint16_t cs;


int power_parse_frame(uint32_t *voltage, uint32_t *current, uint8_t c)
{
   int ret = 0;
   uint32_t data[2];

   assert(voltage);
   assert(current);

   switch(state)
   {
      case READ_SYNC0:
         if (c == (uint8_t)(POWER_PREAMBLE & 0xFF))
            state = READ_SYNC1;
         break;

      case READ_SYNC1:
         if (c == (uint8_t)(POWER_PREAMBLE >> 8))
            state = READ_VOLTAGE0;
         else
            state = READ_SYNC0;
         break;

      case READ_VOLTAGE0:
         *voltage = c;
         state = READ_VOLTAGE1;
         break;

      case READ_VOLTAGE1:
         *voltage |= c << 8;
         state = READ_VOLTAGE2;
         break;

      case READ_VOLTAGE2:
         *voltage |= c << 16;
         state = READ_VOLTAGE3;
         break;

      case READ_VOLTAGE3:
         *voltage |= c << 24;
         state = READ_CURRENT0;
         break;

      case READ_CURRENT0:
         *current = c;
         state = READ_CURRENT1;
         break;

      case READ_CURRENT1:
         *current |= c << 8;
         state = READ_CURRENT2;
         break;
         
      case READ_CURRENT2:
         *current |= c << 16;
         state = READ_CURRENT3;
         break;

      case READ_CURRENT3:
         *current |= c << 24;
         state = READ_CS0;
         break;

      case READ_CS0:
         cs = c;
         state = READ_CS1;
         break;

      case READ_CS1:
         cs |= c << 8;

         data[0] = *voltage;
         data[1] = *current;

         if (cs == checksum((uint8_t *)(data), sizeof(data)))
            ret = 1;
         state = READ_SYNC0;
         break;
   }
   return ret;
}

