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

 Note: This service will terminate after receiving valid GPS information
       and setting the time zone and the system date/time.

 Daylight savings time offsets are computed implicitly by the operating system.

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

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
#include <daemon.h>

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


bool running = true;


int _main(void)
{
   THROW_BEGIN()

   /* init scl and get sockets:: */
   THROW_ON_ERR(scl_init("gpstime"));
   void *scl_socket = scl_get_socket("gps");
   THROW_IF(scl_socket == NULL, -ENODEV);
   sleep(5);

   while (running)
   {
      char buffer[128];
      int ret = scl_recv_static(scl_socket, buffer, sizeof(buffer));
      if (ret > 0)
      {
         msgpack_unpacked msg;
         msgpack_unpacked_init(&msg);
         if (msgpack_unpack_next(&msg, buffer, ret, NULL))
         {
            msgpack_object root = msg.data;
            assert (root.type == MSGPACK_OBJECT_ARRAY);
            int asize = root.via.array.size;
            int fix = gps_msgpack_fix(asize);
            if (fix >= 2)
            {
               float lat = root.via.array.ptr[LAT].via.dec;
               float lon = root.via.array.ptr[LON].via.dec;
               char time_buf[128];
               snprintf(time_buf, root.via.array.ptr[TIME].via.raw.size, "%s", root.via.array.ptr[TIME].via.raw.ptr);
               char cmd[128];
               linux_sys_set_timezone(lat, lon);
               sprintf(cmd, "date -u -s \"%s\"", time_buf);
               if (system(cmd)){};
               break;
            }
         }
         msgpack_unpacked_destroy(&msg);
      }
   }
   THROW_END();
}



void _cleanup(void)
{
   running = false;
}


void main_wrap(int argc, char *argv[])
{
   (void)argc;
   (void)argv;

   exit(-_main());
}


int main(int argc, char *argv[])
{
   char pid_file[1024];
   sprintf(pid_file, "%s/.PenguPilot/run/gpstime.pid", getenv("HOME"));
   daemonize(pid_file, main_wrap, _cleanup, argc, argv);
   return 0;
}

