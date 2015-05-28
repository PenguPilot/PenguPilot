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
  
 Rate Control Proxy Implementation

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

#include <util.h>
#include <service.h>
#include <msgpack_reader.h>
#include <scl.h>


SERVICE_MAIN_BEGIN("rates_sp_proxy", 99)
{
   MSGPACK_PACKER_DECL_INFUNC();
  
   /* open sockets: */
   void *rates_sp_in_socket = scl_get_socket("rates_sp_in", "pull");
   THROW_IF(rates_sp_in_socket == NULL, -EIO);
   void *rates_sp_socket = scl_get_socket("rates_sp", "pub");
   THROW_IF(rates_sp_socket == NULL, -EIO);
   
   MSGPACK_READER_SIMPLE_LOOP_BEGIN(rates_sp_in)
   {
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         float rates_sp[3];
         FOR_N(i, 3)
            rates_sp[i] = root.via.array.ptr[i].via.dec;
         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, 3);
         PACKFV(rates_sp, 3);
         scl_copy_send_dynamic(rates_sp_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
   MSGPACK_READER_SIMPLE_LOOP_END
}
SERVICE_MAIN_END

