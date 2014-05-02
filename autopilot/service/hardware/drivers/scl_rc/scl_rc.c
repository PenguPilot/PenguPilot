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
  
 RC DSL Reader Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <string.h>


#include <util.h>
#include <simple_thread.h>
#include <opcd_interface.h>
#include <pthread.h>
#include <scl.h>
#include <msgpack.h>

#include "scl_rc.h"


#define THREAD_NAME       "scl_rc_reader"
#define THREAD_PRIORITY   96


#define RSSI_MIN RSSI_SCALE(7)


static simple_thread_t thread;
static pthread_mutexattr_t mutexattr;   
static pthread_mutex_t mutex;
static float channels[SCL_MAX_CHANNELS];
static void *scl_socket;
static int sig_valid;


static void scl_read_channels(int *valid, float *_channels)
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
         int n_channels = root.via.array.size - 1;
         *valid = root.via.array.ptr[0].via.i64;
         FOR_N(i, n_channels)
            if (i < SCL_MAX_CHANNELS)
               _channels[i] = root.via.array.ptr[1 + i].via.dec;
      }
      msgpack_unpacked_destroy(&msg);
   }
   else
   {
      msleep(1);
   }
}


SIMPLE_THREAD_BEGIN(thread_func)
{
   float _channels[SCL_MAX_CHANNELS];
   SIMPLE_THREAD_LOOP_BEGIN
   {
      scl_read_channels(&sig_valid, _channels);
      pthread_mutex_lock(&mutex);
      if (sig_valid)
         memcpy(channels, _channels, sizeof(channels));   
      pthread_mutex_unlock(&mutex);
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END



int scl_rc_init(void)
{
   ASSERT_ONCE();
   memset(channels, 0, sizeof(channels));
   scl_socket = scl_get_socket("remote");
   ASSERT_NOT_NULL(scl_socket);
   pthread_mutexattr_init(&mutexattr); 
   pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT); 
   pthread_mutex_init(&mutex, &mutexattr);
   simple_thread_start(&thread, thread_func, THREAD_NAME, THREAD_PRIORITY, NULL);
   return 0;
}


int scl_rc_read(float channels_out[SCL_MAX_CHANNELS])
{
   pthread_mutex_lock(&mutex);
   memcpy(channels_out, channels, sizeof(channels));
   int ret = sig_valid ? 0 : -EAGAIN;
   pthread_mutex_unlock(&mutex);
   return ret;
}

