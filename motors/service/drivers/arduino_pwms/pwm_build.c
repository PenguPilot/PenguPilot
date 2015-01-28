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
  
 Arduino Serial ESC Bridge Frame Builder Implementation

 Copyright (C) 2014 Jan Roemisch, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include <assert.h>

#include "pwm_common.h"
#include "pwm_build.h"
#include "checksum.h"


size_t pwm_build_frame(uint8_t *frame, uint8_t *pwm_data, uint8_t pwm_count)
{
   assert(frame);
   assert(pwm_data);
   assert(pwm_count != 0 && pwm_count <= PWM_COUNT_MAX);
   int i = 0;
   frame[i++] = 0x55;
   assert(pwm_count != 0x55);
   frame[i++] = pwm_count;
   int j;
   for (j = 0; j < pwm_count; j++)
   {
      //assert(pwm_data[i] != 0x55);
      frame[i++] = pwm_data[j];
   }
   uint16_t crc = checksum(pwm_data, pwm_count);
   frame[i++] = (crc >> 8) & 0xFF;
   frame[i++] = crc & 0xFF;
   return i;
}

