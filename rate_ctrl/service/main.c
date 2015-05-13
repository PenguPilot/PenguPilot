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
  
 Rate Control Service Implementation

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <math.h>
#include <simple_thread.h>
#include <service.h>
#include <msgpack_reader.h>
#include "piid.h"


#define SERVICE_NAME "rate_ctrl"
#define SERVICE_PRIO 99


static float rates[3];
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;


/* thread that reads the desired angular rates: */
MSGPACK_READER_BEGIN(rates_reader)
   MSGPACK_READER_LOOP_BEGIN(rates_reader)
   if (root.type == MSGPACK_OBJECT_ARRAY)
   {
      pthread_mutex_lock(&mutex);
      FOR_N(i, 3)
         rates[i] = root.via.array.ptr[i].via.dec;
      pthread_mutex_unlock(&mutex);
   }
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


/* thread that reads the desired integrator status: */
MSGPACK_READER_BEGIN(inten_reader)
   MSGPACK_READER_LOOP_BEGIN(inten_reader)
   bool inten = root.via.i64;
   static bool inten_prev = false;
   if (inten != inten_prev)
   {
      LOG(LL_INFO, "new integrator enable: %d", inten);
      piid_int_enable(inten);
   }
   inten_prev = inten;
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


SERVICE_MAIN_BEGIN
   /* initialize msgpack: */
   msgpack_sbuffer *msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);
   msgpack_packer *pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   THROW_IF(pk == NULL, -ENOMEM);
  
   /* initialize SCL: */
   void *gyro_cal_socket = scl_get_socket("gyro_cal", "sub");
   THROW_IF(gyro_cal_socket == NULL, -EIO);
   void *torques_socket = scl_get_socket("torques", "pub");
   THROW_IF(torques_socket == NULL, -EIO);
   
   MSGPACK_READER_START(rates_reader, "rates", 99);
   MSGPACK_READER_START(inten_reader, "inten", 99);
 
   piid_init(0.005);
   LOG(LL_INFO, "entering main loop");
   simple_thread_t _thread;
   simple_thread_t *thread = &_thread;
   thread->running = true;
   MSGPACK_READER_LOOP_BEGIN(gyro_cal)
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         /* read gyro data: */
         float gyro[3];
         FOR_N(i, 3)
         {
            gyro[i] = root.via.array.ptr[i].via.dec;
         }

         float torques[3];
         /* run rate controller: */
         pthread_mutex_lock(&mutex);
         piid_run(torques, gyro, rates, 0.005);
         pthread_mutex_unlock(&mutex);

         /* send torques: */
         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, 3);
         PACKFV(torques, 3);
         scl_copy_send_dynamic(torques_socket, msgpack_buf->data, msgpack_buf->size);
      }
   MSGPACK_READER_LOOP_END
SERVICE_MAIN_END

