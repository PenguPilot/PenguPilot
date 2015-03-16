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
  
 SCL Power Reader Implementation

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

#include <util.h>
#include <simple_thread.h>
#include <msgpack.h>
#include <scl.h>
#include <logger.h>

#include "scl_power.h"


#define THREAD_PRIORITY 98


static simple_thread_t thread;
static pthread_mutexattr_t mutexattr;
static pthread_mutex_t mutex;
static void *scl_socket;
static float voltage = 16.0;
static float current = 0.4;


static void scl_read_power(float *_voltage, float *_current)
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
         assert(root.via.array.size == 2);
         *_voltage = root.via.array.ptr[0].via.dec;
         *_current = root.via.array.ptr[1].via.dec;
      }
      msgpack_unpacked_destroy(&msg);
   }
   else
   {
      sleep(1);
      LOG(LL_ERROR, "could not read voltage");
   }
}


SIMPLE_THREAD_BEGIN(thread_func)
{
   SIMPLE_THREAD_LOOP_BEGIN
   {
      float voltage_raw = 0.0;
      float current_raw = 0.0;
      scl_read_power(&voltage_raw, &current_raw);
      pthread_mutex_lock(&mutex);
      voltage = voltage_raw;
      current = current_raw;
      pthread_mutex_unlock(&mutex);
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END


int scl_power_init(void)
{
   THROW_BEGIN();
   scl_socket = scl_get_socket("power", "sub");
   THROW_IF(scl_socket == NULL, -ENODEV);
   pthread_mutexattr_init(&mutexattr);
   pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT);
   pthread_mutex_init(&mutex, &mutexattr);
   simple_thread_start(&thread, thread_func, "power_reader", THREAD_PRIORITY, NULL);
   THROW_END();
}


int scl_power_read(float *voltage_out, float *current_out)
{
   pthread_mutex_lock(&mutex);
   *voltage_out = voltage;
   *current_out = current;
   pthread_mutex_unlock(&mutex);
   return 0;
}

