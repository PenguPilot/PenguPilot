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
  
 MAG Calibration Service Implementation

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <msgpack.h>
#include <pthread.h>

#include <scl.h>
#include <service.h>
#include <msgpack_reader.h>
#include <pp_prio.h>

#include <math/euler.h>
#include <math/quat.h>


/* orientation reader thread: */
euler_t euler;
quat_t quat;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

MSGPACK_READER_BEGIN(orientation_reader)
   MSGPACK_READER_LOOP_BEGIN(orientation_reader)
   if (root.type == MSGPACK_OBJECT_ARRAY)
   {
      pthread_mutex_lock(&mutex);
      euler.yaw = root.via.array.ptr[0].via.dec;
      euler.pitch = root.via.array.ptr[1].via.dec;
      euler.roll = root.via.array.ptr[2].via.dec;
      FOR_N(i, 4)
         quat.ve[i] = root.via.array.ptr[3 + i].via.dec;
      pthread_mutex_unlock(&mutex);
   }
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END



SERVICE_MAIN_BEGIN("acc_rot_neu", PP_PRIO_3)
{ 
   quat_init(&quat);

   /* set-up msgpack packer: */
   MSGPACK_PACKER_DECL_INFUNC();
 
   /* open sockets: */
   void *acc_socket = scl_get_socket("acc", "sub");
   THROW_IF(acc_socket == NULL, -EIO);
   void *acc_neu_socket = scl_get_socket("acc_neu", "pub");
   THROW_IF(acc_neu_socket == NULL, -EIO);

   MSGPACK_READER_START(orientation_reader, "orientation", PP_PRIO_3, "sub");
 
   MSGPACK_READER_SIMPLE_LOOP_BEGIN(acc)
   {
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         vec3_t acc;
         vec3_init(&acc);
         FOR_N(i, 3)
            acc.ve[i] = root.via.array.ptr[i].via.dec;
         
         vec3_t acc_neu;
         vec3_init(&acc_neu);
         vec3_t acc_neu2;
         vec3_init(&acc_neu2);

         pthread_mutex_lock(&mutex);
         body_to_neu(&acc_neu, &euler, &acc);
         quat_rot_vec(&acc_neu2, &acc, &quat);
         pthread_mutex_unlock(&mutex);

         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, 3);
         PACKFV(acc_neu.ve, 3);
         scl_copy_send_dynamic(acc_neu_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
   MSGPACK_READER_SIMPLE_LOOP_END;
}
SERVICE_MAIN_END

