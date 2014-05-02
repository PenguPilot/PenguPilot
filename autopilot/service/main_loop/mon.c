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
  
 Monitoring Publisher Implementation

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
#include <scl.h>
#include <pilot.pb-c.h>
#include <periodic_thread.h>

#include "mon.h"

#define THREAD_PRIORITY 96

static pthread_mutexattr_t mutexattr;
static pthread_mutex_t mutex;
static void *mon_socket = NULL;
static MonData mon_data = MON_DATA__INIT;
static periodic_thread_t emitter_thread;


PERIODIC_THREAD_BEGIN(mon_emitter)
{
   PERIODIC_THREAD_LOOP_BEGIN
   {
      MonData _mon_data;
      pthread_mutex_lock(&mutex);
      memcpy(&_mon_data, &mon_data, sizeof(MonData));
      pthread_mutex_unlock(&mutex);

      SCL_PACK_AND_SEND_DYNAMIC(mon_socket, mon_data, _mon_data);
   }
   PERIODIC_THREAD_LOOP_END
}
PERIODIC_THREAD_END


void mon_init(void)
{
   /* open monitoring socket: */
   mon_socket = scl_get_socket("ap_mon");
   ASSERT_NOT_NULL(mon_socket);
   int64_t hwm = 1;
   zmq_setsockopt(mon_socket, ZMQ_SNDHWM, &hwm, sizeof(hwm));

   /* create monitoring connection: */
   const struct timespec period = {0, 100 * NSEC_PER_MSEC};
   pthread_mutexattr_init(&mutexattr);
   pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT);
   pthread_mutex_init(&mutex, &mutexattr);
   periodic_thread_start(&emitter_thread, mon_emitter, "mon_thread", THREAD_PRIORITY, period, NULL);
}


void mon_data_set(float x_err, float y_err, float z_err, float yaw_err)
{
   pthread_mutex_lock(&mutex);
   mon_data.x_err = x_err;
   mon_data.y_err = y_err;
   mon_data.z_err = z_err;
   mon_data.yaw_err = yaw_err;
   pthread_mutex_unlock(&mutex);
}

