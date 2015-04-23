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
#include <msgpack.h>
#include <scl.h>
#include <simple_thread.h>
#include <service.h>

#include "piid.h"


#define SERVICE_NAME "rate_ctrl"
#define SERVICE_PRIO 99


static void *gyro_cal_socket;
static void *torques_socket;
static float rates[3];
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
void *rates_socket;


SIMPLE_THREAD_BEGIN(rates_reader_func)
{
   SIMPLE_THREAD_LOOP_BEGIN
   {
      char buffer[1024];
      int ret = scl_recv_static(rates_socket, buffer, sizeof(buffer));
      if (ret > 0)
      {
         msgpack_unpacked msg;
         msgpack_unpacked_init(&msg);
         if (msgpack_unpack_next(&msg, buffer, ret, NULL))
         {
            msgpack_object root = msg.data;
            if (root.type == MSGPACK_OBJECT_ARRAY)
            {
               pthread_mutex_lock(&mutex);
               FOR_N(i, 3)
                  rates[i] = root.via.array.ptr[i].via.dec;
               pthread_mutex_unlock(&mutex);
            }
         }
         msgpack_unpacked_destroy(&msg);
      }
      else
      {
         msleep(10);   
      }
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END



SERVICE_MAIN_BEGIN
{
   /* initialize msgpack: */
   msgpack_sbuffer *msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);
   msgpack_packer *pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   THROW_IF(pk == NULL, -ENOMEM);
  
   /* initialize SCL: */
   gyro_cal_socket = scl_get_socket("gyro_cal", "sub");
   THROW_IF(gyro_cal_socket == NULL, -EIO);
   torques_socket = scl_get_socket("torques", "pub");
   THROW_IF(torques_socket == NULL, -EIO);
   rates_socket = scl_get_socket("rates", "sub");
   THROW_IF(rates_socket == NULL, -EIO);
   
   /* start rates reader thread: */
   simple_thread_t rates_reader;
   rates_reader.running = false;
   simple_thread_start(&rates_reader, rates_reader_func, "rates_reader", 99, NULL);
 
   piid_init(0.005);

   while (running)
   {
      char buffer[1024];
      int ret = scl_recv_static(gyro_cal_socket, buffer, sizeof(buffer));
      if (ret > 0)
      {
         msgpack_unpacked msg;
         msgpack_unpacked_init(&msg);
         if (msgpack_unpack_next(&msg, buffer, ret, NULL))
         {
            msgpack_object root = msg.data;
            if (root.type == MSGPACK_OBJECT_ARRAY)
            {
               /* read gyro data: */
               float gyro[3];
               FOR_N(i, 3)
                  gyro[i] = root.via.array.ptr[i].via.dec;

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
         }
         msgpack_unpacked_destroy(&msg);
      }
      else
      {
         msleep(10);
      }
   }
}
SERVICE_MAIN_END

