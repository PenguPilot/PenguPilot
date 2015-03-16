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
static msgpack_sbuffer *msgpack_buf = NULL;
static msgpack_packer *pk = NULL;


int _main(void)
{
   THROW_BEGIN()
   
   /* init msgpack buffer: */
   ASSERT_NULL(msgpack_buf);
   msgpack_buf = msgpack_sbuffer_new();
   ASSERT_NOT_NULL(msgpack_buf);
   ASSERT_NULL(pk);
   pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);

   /* init scl and get sockets:: */
   void *gps_socket = scl_get_socket("gps", "sub");
   THROW_IF(gps_socket == NULL, -ENODEV);
   void *ts_socket = scl_get_socket("time_set", "pub");
   THROW_IF(ts_socket == NULL, -ENODEV);
   sleep(5);
   bool set = false;

   while (running)
   {
      char buffer[128];
      int ret = scl_recv_static(gps_socket, buffer, sizeof(buffer));
      if (ret > 0 && !set)
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
               set = true;
            }
         }
         msgpack_unpacked_destroy(&msg);
      }

      msgpack_sbuffer_clear(msgpack_buf);
      if (set)
         msgpack_pack_true(pk);
      else
         msgpack_pack_false(pk);
      scl_copy_send_dynamic(ts_socket, msgpack_buf->data, msgpack_buf->size);
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

