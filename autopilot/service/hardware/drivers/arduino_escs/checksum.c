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
  
 Arduino Protocol Checksum Implementation

 Copyright (C) 2014 Jan Roemisch, Ilmenau University of Technology
 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <assert.h>

#include "checksum.h"


uint16_t checksum(uint8_t *data, uint8_t size)
{
   /* taken from: http://stackoverflow.com/questions/10564491/function-to-calculate-a-crc16-checksum */
   #define CRC16 0x8005
   assert(data);
   uint16_t out = 0;
   int bits_read = 0, bit_flag;

   while(size > 0)
   {
      bit_flag = out >> 15;

      /* Get next bit: */
      out <<= 1;
      out |= (*data >> bits_read) & 1; // item a) work from the least significant bits

      /* Increment bit counter: */
      bits_read++;
      if(bits_read > 7)
      {
         bits_read = 0;
         data++;
         size--;
      }

      /* Cycle check: */
      if(bit_flag)
         out ^= CRC16;
   }

   // item b) "push out" the last 16 bits
   int i;
   for (i = 0; i < 16; ++i) {
      bit_flag = out >> 15;
      out <<= 1;
      if(bit_flag)
         out ^= CRC16;
   }

   // item c) reverse the bits
   uint16_t crc = 0;
   i = 0x8000;
   int j = 0x0001;
   for (; i != 0; i >>=1, j <<= 1) {
      if (i & out) crc |= j;
   }

   uint8_t high = crc >> 8;
   uint8_t low = crc & 0xFF;
   if (high == 0xFF)
      high = 0;   
   if (low == 0xFF)
      low = 0;   
   return (high << 8) | low;
}

