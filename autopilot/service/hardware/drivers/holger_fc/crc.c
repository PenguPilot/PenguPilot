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
  
 Mikrokopter CRC Implementation
 
 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "crc.h"


crc_t calc_crc(const char *data, const unsigned int len)
{
   unsigned int i;
   crc_t crc = 0;
   for (i = 0; i < len; i++)
   {
      crc = crc + data[i];
   }
   crc %= 4096;
   return crc;
}


int crc_ok(const crc_t crc, const char *crc_chars)
{
   char hcrc = '=' + crc / 64;
   char lcrc = '=' + crc % 64;
   return    hcrc == crc_chars[0]
             && lcrc == crc_chars[1];
}


int calc_crc_chars(char *crc_chars, const crc_t crc)
{
   crc_chars[0] = '=' + crc / 64;
   crc_chars[1] = '=' + crc % 64;
   return 2;
}

