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
  
 Linux System Misc Implementation

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <math.h>
#include <stdlib.h>

#include "linux_sys.h"
#include "tz_lookup.h"


void linux_sys_set_timezone(float lat_dest, float lon_dest)
{
   char tzname[1024];
   float dist_max = 360.0f;
   int i;
   for (i = 0; i < TZ_LOOKUP_ENTRIES; i++) 
   {
      float lat = tz_lookup[i].lat;
      float lon = tz_lookup[i].lon;
      float dist = fabs(lon - lon_dest) + fabs(lat - lat_dest);
      if (dist < dist_max)
      {
         dist_max = dist;
         strcpy(tzname, tz_lookup[i].tz);
      }
   }
   char cmd[1024];
   sprintf(cmd, "cp /usr/share/zoneinfo/%s /etc/localtime", tzname);
   if (system(cmd)){};
}

