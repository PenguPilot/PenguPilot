
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
  
 Mikrokopter Data Formats Interface
 
 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __MK_DATA_FORMATS_H__
#define __MK_DATA_FORMATS_H__


struct ExternControl
{
   unsigned char digital[2];
   unsigned char remote_keys;
   signed char pitch;
   signed char roll;
   signed char yaw;
   unsigned char gas;
   signed char height;
   unsigned char free;
   unsigned char frame;
   unsigned char config;
} __attribute__((packed));


typedef struct
{
   unsigned char digital[2];
   short int analog[32];
}
fc_debug_t;


#endif /* __MK_DATA_FORMATS_H__ */
