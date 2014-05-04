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
  
 SCL GPS Data Reader Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <util.h>
#include <simple_thread.h>
#include <scl.h>
#include <interval.h>
#include <gps_msgpack.h>
#include <msgpack.h>

#include "scl_gps.h"


#define THREAD_PRIORITY 98



/* if we receive no valid GPS fix for more than this amount of time,
   indicate an error;
   this condition applies if:
     - the gps publisher does not send data (driver or bus problem)
     - the gps publisher sends data but the fix is not 2D/3D */


#define GPS_TIMEOUT (5.0f)


static gps_data_t gps_data = {FIX_NOT_SEEN, 0, 0, 0, 0, 0, 0};
static simple_thread_t thread;
static void *scl_socket;
static pthread_mutexattr_t mutexattr;
static pthread_mutex_t mutex;
static interval_t interval;
static float timer = 0.0f;


SIMPLE_THREAD_BEGIN(thread_func)
{
   SIMPLE_THREAD_LOOP_BEGIN
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
            pthread_mutex_lock(&mutex);
            gps_data.fix = fix(asize);
            if (gps_data.fix >= 2)
            {
               timer = 0.0f; /* reset timer */
               gps_data.sats = root.via.array.ptr[SATS].via.i64;
               gps_data.lat = root.via.array.ptr[LAT].via.dec;
               gps_data.lon = root.via.array.ptr[LON].via.dec;
               gps_data.alt = root.via.array.ptr[ALT].via.dec;
               gps_data.course = root.via.array.ptr[COURSE].via.dec;
               gps_data.speed = root.via.array.ptr[SPEED].via.dec;
               if (gps_data.fix == 3)
                  gps_data.alt = root.via.array.ptr[ALT].via.dec;
            }
            pthread_mutex_unlock(&mutex);
         }
         msgpack_unpacked_destroy(&msg);
      }
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END


int scl_gps_init(void)
{
   ASSERT_ONCE();
   THROW_BEGIN();
   scl_socket = scl_get_socket("gps");
   THROW_IF(scl_socket == NULL, -ENODEV);
   pthread_mutexattr_init(&mutexattr);
   pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT);
   pthread_mutex_init(&mutex, &mutexattr);
   interval_init(&interval);
   simple_thread_start(&thread, thread_func, "gps_reader", THREAD_PRIORITY, NULL);
   THROW_END();
}


int scl_gps_read(gps_data_t *data_out)
{
   ASSERT_NOT_NULL(data_out);
   int ret_code = 0;
   pthread_mutex_lock(&mutex);
   timer += interval_measure(&interval);
   if (timer > GPS_TIMEOUT)
   {
      ret_code = -1;
   }
   else
   {
      *data_out = gps_data;
   }
   pthread_mutex_unlock(&mutex);
   return ret_code;
}

