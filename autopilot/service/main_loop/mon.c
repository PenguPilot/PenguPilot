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

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

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
#include <msgpack.h>


#define THREAD_PRIORITY 96


static pthread_mutexattr_t mutexattr;
static pthread_mutex_t mutex;
static float n = 0.0f;
static float e = 0.0f;
static float u_ground = 0.0f;
static float u = 0.0f;
static float y = 0.0f;
static float n_err = 0.0f;
static float e_err = 0.0f;
static float u_err = 0.0f;
static float y_err = 0.0f;
static void *mon_socket = NULL;
static periodic_thread_t emitter_thread;
static msgpack_sbuffer *msgpack_buf = NULL;
static msgpack_packer *pk = NULL;


PERIODIC_THREAD_BEGIN(mon_emitter)
{
   PERIODIC_THREAD_LOOP_BEGIN
   {
      msgpack_sbuffer_clear(msgpack_buf);
      msgpack_pack_array(pk, 9);

      pthread_mutex_lock(&mutex);
      PACKF(n); /* 0 */
      PACKF(e); /* 1 */
      PACKF(u_ground); /* 2 */
      PACKF(u); /* 3 */
      PACKF(y); /* 4 */
      PACKF(n_err); /* 5 */
      PACKF(e_err); /* 6 */
      PACKF(u_err); /* 7 */
      PACKF(y_err); /* 8 */
      pthread_mutex_unlock(&mutex);
      
      //if (n != 0.0f && e != 0.0f)
      {
         scl_copy_send_dynamic(mon_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
   PERIODIC_THREAD_LOOP_END
}
PERIODIC_THREAD_END


void mon_init(void)
{
   ASSERT_ONCE();

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

   /* init msgpack buffer: */
   ASSERT_NULL(msgpack_buf);
   msgpack_buf = msgpack_sbuffer_new();
   ASSERT_NOT_NULL(msgpack_buf);
   ASSERT_NULL(pk);
   pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);

   periodic_thread_start(&emitter_thread, mon_emitter, "mon_thread", THREAD_PRIORITY, period, NULL);
}


void mon_data_set(float _n, float _e, float _u_ground, float _u, float _y,
                  float _n_err, float _e_err, float _u_err, float _y_err)
{
   pthread_mutex_lock(&mutex);
   n = _n;
   e = _e;
   u_ground = _u_ground;
   u = _u;
   y = _y;
   n_err = _n_err;
   e_err = _e_err;
   u_err = _u_err;
   y_err = _y_err;
   pthread_mutex_unlock(&mutex);
}

