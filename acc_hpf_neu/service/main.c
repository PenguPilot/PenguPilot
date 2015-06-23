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
  
 World ACC High-Pass Filter Service Implementation

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

#include <scl.h>
#include <service.h>
#include <msgpack_reader.h>


SERVICE_MAIN_BEGIN("acc_hpf_neu", 99)
{ 
   /* set-up msgpack packer: */
   MSGPACK_PACKER_DECL_INFUNC();
 
   /* open sockets: */
   void *acc_neu_socket = scl_get_socket("acc_neu", "sub");
   THROW_IF(acc_neu_socket == NULL, -EIO);
   void *acc_hp_neu_socket = scl_get_socket("acc_hp_neu", "pub");
   THROW_IF(acc_hp_neu_socket == NULL, -EIO);

   /* filter states: */
   float fs_x = 0.0;
   float fs_y = 0.0;
   float fs_z = 9.81;

   MSGPACK_READER_SIMPLE_LOOP_BEGIN(acc_neu)
   {
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         float x = root.via.array.ptr[0].via.dec;
         float y = root.via.array.ptr[1].via.dec;
         float z = root.via.array.ptr[2].via.dec;
         
         /* low-pass to determine "dc"-offset: */
         float a = 0.001; /* TODO: make dt dependent! */
         fs_x = fs_x * (1.0 - a) + x * a;
         fs_y = fs_y * (1.0 - a) + y * a;
         fs_z = fs_z * (1.0 - a) + z * a;

         /* high-pass: */
         x -= fs_x;
         y -= fs_y;
         z -= fs_z;

         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, 3);
         PACKF(x);
         PACKF(y);
         PACKF(z);
         scl_copy_send_dynamic(acc_hp_neu_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
   MSGPACK_READER_SIMPLE_LOOP_END;
}
SERVICE_MAIN_END

