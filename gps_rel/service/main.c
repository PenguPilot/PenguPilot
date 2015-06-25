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
  
 GPS to Relative Coordinates Converter Implementation

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
#include <service.h>
#include <msgpack_reader.h>
#include <gps_msgpack.h>
#include <math/conv.h>
#include <pp_prio.h>


static void *gps_socket = NULL;
static void *gps_rel_socket = NULL;
static double start_lat = 0.0;
static double start_lon = 0.0;


SERVICE_MAIN_BEGIN("gps_rel", PP_PRIO_3)
{ 
   /* set-up msgpack packer: */
   MSGPACK_PACKER_DECL_INFUNC();
 
   /* open sockets: */
   gps_socket = scl_get_socket("gps", "sub");
   THROW_IF(gps_socket == NULL, -EIO);
   gps_rel_socket = scl_get_socket("gps_rel", "pub");
   THROW_IF(gps_rel_socket == NULL, -EIO);

   MSGPACK_READER_SIMPLE_LOOP_BEGIN(gps)
   {
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         if (gps_msgpack_fix(root.via.array.size) >= 2)
         {
            /* array access: */
            double lat = root.via.array.ptr[LAT].via.dec;
            double lon = root.via.array.ptr[LON].via.dec;
            float speed = root.via.array.ptr[SPEED].via.dec;
            float course = root.via.array.ptr[COURSE].via.dec;

            /* set start latitude/longitude, if needed: */
            if (start_lat == 0.0 && start_lon == 0.0)
            {
               start_lat = lat;
               start_lon = lon;
               LOG(LL_INFO, "start lat: %f, lon: %f", start_lat, start_lon);
            }

            /* transformations: */
            float x = (lat - start_lat) / 8.983152841195214e-06;
            float y = (lon - start_lon) / 8.983152841195214e-06 * cos(M_PI / 180.0f * start_lat);
            float speed_mps = speed / 3.6; /* km/h to m/s */
            float alpha = deg2rad(course);
            float vx = speed_mps * cos(alpha);
            float vy = speed_mps * sin(alpha);

            /* send data: */
            msgpack_sbuffer_clear(msgpack_buf);
            msgpack_pack_array(pk, 4);
            PACKF(x);
            PACKF(vx);
            PACKF(y);
            PACKF(vy);
            scl_copy_send_dynamic(gps_rel_socket, msgpack_buf->data, msgpack_buf->size);
         }
      }
   }
   MSGPACK_READER_SIMPLE_LOOP_END;
}
SERVICE_MAIN_END

