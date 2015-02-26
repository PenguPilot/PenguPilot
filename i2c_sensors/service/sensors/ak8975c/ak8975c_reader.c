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
  
 AK8975C Reader Implementation

 Copyright (C) 2014 Jan Roemisch, Integrated Communication Systems Group, TU Ilmenau
 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <errno.h>

#include <util.h>
#include <simple_thread.h>
#include <threadsafe_types.h>

#include "ak8975c_reader.h"
#include "ak8975c.h"


#define THREAD_NAME       "ak8975c_reader"
#define THREAD_PRIORITY   98


static simple_thread_t thread;
static pthread_mutexattr_t mutexattr;   
static pthread_mutex_t mutex;
static ak8975c_dev_t ak8975c;
static vec3_t val;
static vec3_t null_val;
static int status = -EAGAIN;


SIMPLE_THREAD_BEGIN(thread_func)
{
   SIMPLE_THREAD_LOOP_BEGIN
   {
      int ret = ak8975c_read(&ak8975c);
      pthread_mutex_lock(&mutex);
      status = ret;
      vec_copy(&val, &ak8975c.raw);
      pthread_mutex_unlock(&mutex);
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END


int ak8975c_reader_init(i2c_bus_t *bus)
{
   ASSERT_ONCE();
   THROW_BEGIN();

   vec3_init(&val);
   vec3_init(&null_val);
   
   pthread_mutexattr_init(&mutexattr); 
   pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT); 
   pthread_mutex_init(&mutex, &mutexattr);
   
   THROW_ON_ERR(ak8975c_init(&ak8975c, bus));
   
   simple_thread_start(&thread, thread_func, THREAD_NAME, THREAD_PRIORITY, NULL);
   THROW_END();
}


int ak8975c_reader_get(vec3_t *mag)
{
   pthread_mutex_lock(&mutex);
   if (status < 0)
      vec_copy(mag, &null_val);
   else
      vec_copy(mag, &val);
   int ret = status;
   pthread_mutex_unlock(&mutex);
   return ret;
}

