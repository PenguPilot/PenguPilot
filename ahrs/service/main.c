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
#include <pp_prio.h>
#include <marg_data.h>


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


#include "mahony_ahrs.h"


SERVICE_MAIN_BEGIN("ahrs", PP_PRIO_2)
{
   tsfloat_init(&decl, 0.0f);

   /* init SCL: */
   void *gyro_socket = scl_get_socket("gyro", "sub");
   THROW_IF(gyro_socket == NULL, -EIO);
   void *orientation_socket = scl_get_socket("orientation", "pub");
   THROW_IF(orientation_socket == NULL, -EIO);
 
   LOG(LL_INFO, "starting threads");
   marg_data_init(&marg_data);

   /* start reader threads: */
   MSGPACK_READER_START(acc_reader, "acc", PP_PRIO_2, "sub");
   MSGPACK_READER_START(mag_reader, "mag", PP_PRIO_2, "sub");
   MSGPACK_READER_START(decl_reader, "decl", PP_PRIO_2, "sub");
 
   /* set-up msgpack packer: */
   MSGPACK_PACKER_DECL_INFUNC();
 
   interval_t interval;
   interval_init(&interval);


   float init = false;
   tsfloat_t p_end;
   tsfloat_t p_start;
   tsfloat_t p_step;
   tsfloat_t i_end;
   
   /* read configuration: */
   opcd_param_t params[] =
   {
      {"p_start", &p_start},
      {"p_step", &p_step},
      {"p_end", &p_end},
      {"i", &i_end},
      OPCD_PARAMS_END
   };
   opcd_params_apply(".", params);

   mahony_ahrs_t ahrs;
   mahony_ahrs_init(&ahrs, tsfloat_get(&p_start), tsfloat_get(&i_end));
   mahony_ahrs_t imu;
   mahony_ahrs_init(&imu, tsfloat_get(&p_start), tsfloat_get(&i_end));
   
   LOG(LL_INFO, "started decreasing beta gain");
   MSGPACK_READER_SIMPLE_LOOP_BEGIN(gyro)
   {
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         ahrs.twoKi = imu.twoKi = tsfloat_get(&i_end);
         ahrs.twoKp -= tsfloat_get(&p_step);
         if (ahrs.twoKp < tsfloat_get(&p_end) || init)
         {
            init = true;
            ahrs.twoKp = imu.twoKp = tsfloat_get(&p_end);
         }
         float dt = interval_measure(&interval);
         pthread_mutex_lock(&mutex);
         FOR_N(i, 3)
            marg_data.gyro.ve[i] = root.via.array.ptr[i].via.dec;
         
         mahony_ahrs_update_imu(&imu, marg_data.gyro.x,
                            marg_data.gyro.y,
                            marg_data.gyro.z,
                            marg_data.acc.x,
                            marg_data.acc.y,
                            marg_data.acc.z, dt);
        
         mahony_ahrs_update(&ahrs, marg_data.gyro.x,
                            marg_data.gyro.y,
                            marg_data.gyro.z,
                            marg_data.acc.x,
                            marg_data.acc.y,
                            marg_data.acc.z,
                            marg_data.mag.x,
                            marg_data.mag.y,
                            marg_data.mag.z, dt);

         quat_t rot_quat;
         quat_init_axis(&rot_quat, 0, 0, -1.0, tsfloat_get(&decl));
         quat_t quat;
         quat_init(&quat);
         quat_mul(&quat, &ahrs.quat, &rot_quat);

         euler_t ahrs_euler;
         quat_to_euler(&ahrs_euler, &quat);
         
         euler_t imu_euler;
         quat_to_euler(&imu_euler, &imu.quat);

         euler_t euler;
         euler.pitch = imu_euler.pitch;
         euler.roll = imu_euler.roll;
         euler.yaw = ahrs_euler.yaw;
         
         pthread_mutex_unlock(&mutex);
         if (!init)
         {
            EVERY_N_TIMES(1000, LOG(LL_INFO, "orientation (y p r): %f %f %f; declination: %f", 
                                   rad2deg(euler.yaw), rad2deg(euler.pitch), rad2deg(euler.roll), deg2rad(tsfloat_get(&decl))));
         }
         else
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

