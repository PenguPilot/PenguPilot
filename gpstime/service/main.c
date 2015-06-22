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
  
 GPS Time Zone and Data/Time Setting Service

 Daylight savings time offsets are computed by the operating system.

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <unistd.h>
#include <math.h>
#include <stdbool.h>
#include <msgpack.h>

#include <gps_msgpack.h>
#include <util.h>
#include <scl.h>
#include <threadsafe_types.h>
#include <service.h>
#include <logger.h>
#include <msgpack_reader.h>

#include "tz_lookup.h"


static void linux_sys_set_timezone(float lat_dest, float lon_dest)
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


SERVICE_MAIN_BEGIN("gpstime", 0)
{
   /* set-up msgpack packer: */
   MSGPACK_PACKER_DECL_INFUNC();
 
   /* init scl and get sockets: */
   void *gps_socket = scl_get_socket("gps", "sub");
   THROW_IF(gps_socket == NULL, -ENODEV);
   void *ts_socket = scl_get_socket("time_set", "pub");
   THROW_IF(ts_socket == NULL, -ENODEV);
   bool set = false;
   
   MSGPACK_READER_SIMPLE_LOOP_BEGIN(gps)
   {
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         int asize = root.via.array.size;
         int fix = gps_msgpack_fix(asize);
         if (fix >= 2)
         {
            float lat = root.via.array.ptr[LAT].via.dec;
            float lon = root.via.array.ptr[LON].via.dec;
            char time_buf[128];
            snprintf(time_buf, root.via.array.ptr[TIME].via.raw.size, "%s", root.via.array.ptr[TIME].via.raw.ptr);
            char cmd[128];
            if (!set)
            {
               linux_sys_set_timezone(lat, lon);
               LOG(LL_INFO, "setting UTC date/time to: %s", time_buf);
               sprintf(cmd, "date -u -s \"%s\"", time_buf);
               if (system(cmd)){};
               set = true;
            }
         }
      }
      msgpack_sbuffer_clear(msgpack_buf);
      if (set)
         msgpack_pack_true(pk);
      else
         msgpack_pack_false(pk);
      scl_copy_send_dynamic(ts_socket, msgpack_buf->data, msgpack_buf->size);
   }
   MSGPACK_READER_SIMPLE_LOOP_END
}
SERVICE_MAIN_END

