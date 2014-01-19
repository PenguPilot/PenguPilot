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


#include <util.h>
#include <scl.h>
#include <pilot.pb-c.h>
#include <periodic_thread.h>

#include "mon.h"


static pthread_mutex_t mon_data_mutex = PTHREAD_MUTEX_INITIALIZER;
static void *mon_socket = NULL;
static MonData mon_data = MON_DATA__INIT;
static periodic_thread_t emitter_thread;


PERIODIC_THREAD_BEGIN(mon_emitter)
{
   PERIODIC_THREAD_LOOP_BEGIN
   {
      pthread_mutex_lock(&mon_data_mutex);
      SCL_PACK_AND_SEND_DYNAMIC(mon_socket, mon_data, mon_data);
      pthread_mutex_unlock(&mon_data_mutex);
   }
   PERIODIC_THREAD_LOOP_END
}
PERIODIC_THREAD_END


void mon_init(void)
{
   /* open monitoring socket: */
   mon_socket = scl_get_socket("mon");
   ASSERT_NOT_NULL(mon_socket);
   int64_t hwm = 1;
   zmq_setsockopt(mon_socket, ZMQ_SNDHWM, &hwm, sizeof(hwm));

   /* create monitoring connection: */
   const struct timespec period = {0, 100 * NSEC_PER_MSEC};
   periodic_thread_start(&emitter_thread, mon_emitter, "mon_thread", 0, period, NULL);
}


void mon_data_set(float x_err, float y_err, float z_err, float yaw_err)
{
   if (pthread_mutex_trylock(&mon_data_mutex) == 0)
   {
      mon_data.x_err = x_err;
      mon_data.y_err = y_err;
      mon_data.z_err = z_err;
      mon_data.yaw_err = yaw_err;
      pthread_mutex_unlock(&mon_data_mutex);
   }
}

