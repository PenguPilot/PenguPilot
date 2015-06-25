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
  
 Rotation Rate Control Service Implementation

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
#include <threadsafe_types.h>
#include <msgpack_reader.h>
#include <gyro.h>

#include "piid.h"

#define RATE_CTRL_PITCH 0
#define RATE_CTRL_ROLL  1
#define RATE_CTRL_YAW   2


static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static tsfloat_t rs_ctrl_sp_p;
static tsfloat_t rs_ctrl_sp_r;
static tsfloat_t rs_ctrl_sp_y;


/* reads desired pitch rate: */
MSGPACK_READER_BEGIN(rs_ctrl_sp_p_reader)
   MSGPACK_READER_LOOP_BEGIN(rs_ctrl_sp_p_reader)
      tsfloat_set(&rs_ctrl_sp_p, root.via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


/* reads desired roll rate: */
MSGPACK_READER_BEGIN(rs_ctrl_sp_r_reader)
   MSGPACK_READER_LOOP_BEGIN(rs_ctrl_sp_r_reader)
      tsfloat_set(&rs_ctrl_sp_r, root.via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


/* reads desired yaw rate: */
MSGPACK_READER_BEGIN(rs_ctrl_sp_y_reader)
   MSGPACK_READER_LOOP_BEGIN(rs_ctrl_sp_y_reader)
      tsfloat_set(&rs_ctrl_sp_y, root.via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


/* thread that reads the desired integrator status: */
MSGPACK_READER_BEGIN(int_en_reader)
   MSGPACK_READER_LOOP_BEGIN(int_en_reader)
   piid_int_enable(root.via.i64);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


/* thread that reads the desired integrator status: */
MSGPACK_READER_BEGIN(int_reset_reader)
   MSGPACK_READER_LOOP_BEGIN(int_reset_reader)
   if (root.via.i64)
   {
      pthread_mutex_lock(&mutex);
      piid_reset();
      pthread_mutex_unlock(&mutex);
   }
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


SERVICE_MAIN_BEGIN("rs_ctrl", 99)
{
   /* safe initial values: */
   tsfloat_init(&rs_ctrl_sp_p, 0.0);
   tsfloat_init(&rs_ctrl_sp_r, 0.0);
   tsfloat_init(&rs_ctrl_sp_y, 0.0);

   /* initialize msgpack: */
   msgpack_sbuffer *msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);
   msgpack_packer *pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   THROW_IF(pk == NULL, -ENOMEM);
  
   /* initialize SCL: */
   void *gyro_socket = scl_get_socket("gyro", "sub");
   THROW_IF(gyro_socket == NULL, -EIO);
   void *torques_socket = scl_get_socket("torques", "pub");
   THROW_IF(torques_socket == NULL, -EIO);
   
   MSGPACK_READER_START(rs_ctrl_sp_p_reader, "rs_ctrl_sp_p", 99, "sub");
   MSGPACK_READER_START(rs_ctrl_sp_r_reader, "rs_ctrl_sp_r", 99, "sub");
   MSGPACK_READER_START(rs_ctrl_sp_y_reader, "rs_ctrl_sp_y", 99, "sub");

   MSGPACK_READER_START(int_en_reader, "int_en", 99, "sub");
   MSGPACK_READER_START(int_reset_reader, "int_reset", 99, "sub");
 
   const float sample_dt = 0.005;
   piid_init(sample_dt);
   LOG(LL_INFO, "entering main loop");

   MSGPACK_READER_SIMPLE_LOOP_BEGIN(gyro)
   {
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         /* read synchronous gyro data: */
         float gyro[3];
         gyro[GYRO_X_AXIS] = root.via.array.ptr[GYRO_X_DIR].via.dec;
         gyro[GYRO_Y_AXIS] = root.via.array.ptr[GYRO_Y_DIR].via.dec;
         gyro[GYRO_Z_AXIS] = root.via.array.ptr[GYRO_Z_DIR].via.dec;

         /* retrieve asynchronous setpoints: */
         float rates[3] = {tsfloat_get(&rs_ctrl_sp_p),
                           tsfloat_get(&rs_ctrl_sp_r),
                           tsfloat_get(&rs_ctrl_sp_y)};
         
         /* run rate controller: */
         float torques[3];
         pthread_mutex_lock(&mutex);
         piid_run(torques, gyro, rates, sample_dt);
         pthread_mutex_unlock(&mutex);

         /* send synchronous torques: */
         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, 3);
         PACKFV(torques, 3);
         scl_copy_send_dynamic(torques_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
   MSGPACK_READER_SIMPLE_LOOP_END
}
SERVICE_MAIN_END

