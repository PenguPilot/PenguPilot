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
  
 World Position Estimator Service Implementation

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
#include <interval.h>
#include <threadsafe_types.h>
#include <pp_prio.h>

#include "pos.h"


tsfloat_t n;
tsfloat_t vn;
tsfloat_t e;
tsfloat_t ve;
MSGPACK_READER_BEGIN(gps_rel_reader)
   MSGPACK_READER_LOOP_BEGIN(gps_rel_reader)
      tsfloat_set(&n, root.via.array.ptr[0].via.dec);
      tsfloat_set(&vn, root.via.array.ptr[1].via.dec);
      tsfloat_set(&e, root.via.array.ptr[2].via.dec);
      tsfloat_set(&ve, root.via.array.ptr[3].via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


tsfloat_t ultra;
MSGPACK_READER_BEGIN(ultra_reader)
   MSGPACK_READER_LOOP_BEGIN(ultra_reader)
      tsfloat_set(&ultra, root.via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


tsfloat_t baro;
MSGPACK_READER_BEGIN(baro_reader)
   MSGPACK_READER_LOOP_BEGIN(baro_reader)
      /* array 0: altitude, array 1: temperature */
      tsfloat_set(&baro, root.via.array.ptr[0].via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END



SERVICE_MAIN_BEGIN("pos_speed_est_neu", PP_PRIO_3)
{ 
   tsfloat_init(&n, 0.0);
   tsfloat_init(&vn, 0.0);
   tsfloat_init(&e, 0.0);
   tsfloat_init(&ve, 0.0);
   tsfloat_init(&ultra, 0.2);

   pos_init();
   /* set-up msgpack packer: */
   MSGPACK_PACKER_DECL_INFUNC();
 
   /* open sockets: */
   void *acc_world_hp_socket = scl_get_socket("acc_hp_neu", "sub");
   THROW_IF(acc_world_hp_socket == NULL, -EIO);
   void *pos_speed_est_socket = scl_get_socket("pos_speed_est_neu", "pub");
   THROW_IF(pos_speed_est_socket == NULL, -EIO);

   MSGPACK_READER_START(gps_rel_reader, "gps_rel", PP_PRIO_3, "sub");
   MSGPACK_READER_START(ultra_reader, "ultra_raw", PP_PRIO_3, "sub");
   MSGPACK_READER_START(baro_reader, "baro_raw", PP_PRIO_3, "sub");
 
   interval_t interval;
   interval_init(&interval);

   MSGPACK_READER_SIMPLE_LOOP_BEGIN(acc_world_hp)
   {
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         pos_t pos;
         pos_in_t pos_in; /* contains: dt, ultra_u, baro_u, pos_n, pos_e, speed_n, speed_e, acc */
         vec3_init(&pos_in.acc);
         pos_in.dt = interval_measure(&interval);
         pos_in.ultra_u = tsfloat_get(&ultra);
         pos_in.baro_u = tsfloat_get(&baro);
         pos_in.pos_n = tsfloat_get(&n);
         pos_in.speed_n = tsfloat_get(&vn);
         pos_in.pos_e = tsfloat_get(&e);
         pos_in.speed_e = tsfloat_get(&ve);
         FOR_N(i, 3)
            pos_in.acc.ve[i] = root.via.array.ptr[i].via.dec;
         
         /* run position estimate: */
         pos_update(&pos, &pos_in);
         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, 8);
         PACKF(pos.ultra_u.pos);
         PACKF(pos.ultra_u.speed);
         PACKF(pos.baro_u.pos);
         PACKF(pos.baro_u.speed);
         PACKF(pos.ne_pos.n);
         PACKF(pos.ne_speed.n);
         PACKF(pos.ne_pos.e);
         PACKF(pos.ne_speed.e);
         scl_copy_send_dynamic(pos_speed_est_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
   MSGPACK_READER_SIMPLE_LOOP_END;
}
SERVICE_MAIN_END

