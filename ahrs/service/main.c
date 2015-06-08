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
  
 AHRS Service Implementation

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <threadsafe_types.h>
#include <simple_thread.h>
#include <interval.h>
#include <math/conv.h>
#include <service.h>
#include <msgpack_reader.h>

#include "cal_ahrs.h"


static marg_data_t marg_data;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;


/* accelerometer reader thread: */
MSGPACK_READER_BEGIN(acc_reader)
   MSGPACK_READER_LOOP_BEGIN(acc_reader)
   if (root.type == MSGPACK_OBJECT_ARRAY)
   {
      pthread_mutex_lock(&mutex);
      FOR_N(i, 3)
         marg_data.acc.ve[i] = root.via.array.ptr[i].via.dec;
      pthread_mutex_unlock(&mutex);
   }
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


/* magnetometer reader thread: */
MSGPACK_READER_BEGIN(mag_reader)
   MSGPACK_READER_LOOP_BEGIN(mag_reader)
   if (root.type == MSGPACK_OBJECT_ARRAY)
   {
      pthread_mutex_lock(&mutex);
      FOR_N(i, 3)
         marg_data.mag.ve[i] = root.via.array.ptr[i].via.dec;
      pthread_mutex_unlock(&mutex);
   }
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


/* declination reader thread: */
tsfloat_t decl;
MSGPACK_READER_BEGIN(decl_reader)
   MSGPACK_READER_LOOP_BEGIN(decl_reader)
   tsfloat_set(&decl, root.via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END

//#undef SERVICE_MAIN_DEBUG
//#define SERVICE_MAIN_DEBUG true

SERVICE_MAIN_BEGIN("ahrs", 99)
{
   tsfloat_init(&decl, 0.0f);

   /* init SCL: */
   void *gyro_socket = scl_get_socket("gyro_cal", "sub");
   THROW_IF(gyro_socket == NULL, -EIO);
   void *orientation_socket = scl_get_socket("orientation", "pub");
   THROW_IF(orientation_socket == NULL, -EIO);
 
   LOG(LL_INFO, "starting threads");
   marg_data_init(&marg_data);

   /* start reader threads: */
   MSGPACK_READER_START(acc_reader, "acc_cal", 99, "sub");
   MSGPACK_READER_START(mag_reader, "mag_cal", 99, "sub");
   MSGPACK_READER_START(decl_reader, "decl", 99, "sub");
 
   /* init cal ahrs:*/
   cal_ahrs_init();
   
   /* set-up msgpack packer: */
   MSGPACK_PACKER_DECL_INFUNC();
 
   interval_t interval;
   interval_init(&interval);

   LOG(LL_INFO, "started decreasing beta gain");
   MSGPACK_READER_SIMPLE_LOOP_BEGIN(gyro)
   {
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         float dt = interval_measure(&interval);
         pthread_mutex_lock(&mutex);
         FOR_N(i, 3)
            marg_data.gyro.ve[i] = root.via.array.ptr[i].via.dec;
         euler_t euler;
         int ahrs_state = cal_ahrs_update(&euler, &marg_data, tsfloat_get(&decl), dt);
         pthread_mutex_unlock(&mutex);
         if (ahrs_state == 0)
         {
            EVERY_N_TIMES(100, LOG(LL_INFO, "orientation (y p r): %f %f %f; declination: %f", 
                                   rad2deg(euler.yaw), rad2deg(euler.pitch), rad2deg(euler.roll), deg2rad(tsfloat_get(&decl))));
         }
         else if (ahrs_state == 1)
         {
            ONCE(LOG(LL_INFO, "final beta gain reached, final orientation (y p r): %.1f %.1f %.1f",
                     rad2deg(euler.yaw), rad2deg(euler.pitch), rad2deg(euler.roll)));
            EVERY_N_TIMES(1000, LOG(LL_INFO, "orientation (y p r): %f %f %f; declination: %f", 
                                   rad2deg(euler.yaw), rad2deg(euler.pitch), rad2deg(euler.roll), rad2deg(tsfloat_get(&decl))));
            msgpack_sbuffer_clear(msgpack_buf);
            msgpack_pack_array(pk, 3);
            PACKF(euler.yaw);
            PACKF(euler.pitch);
            PACKF(euler.roll);
            scl_copy_send_dynamic(orientation_socket, msgpack_buf->data, msgpack_buf->size);
         }
      }
   }
   MSGPACK_READER_SIMPLE_LOOP_END
}
SERVICE_MAIN_END

