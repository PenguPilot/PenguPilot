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


#include <unistd.h>
#include <msgpack.h>

#include <daemon.h>
#include <util.h>
#include <scl.h>
#include <opcd_interface.h>
#include <serial.h>
#include <threadsafe_types.h>
#include <interval.h>
#include <logger.h>
#include <math/conv.h>

#include "scl_mag_decl.h"
#include "cal_ahrs.h"


static int running = 1;
static char *name = "ahrs";


int _main(void)
{
   ASSERT_ONCE();
   THROW_BEGIN();
 
   /* init SCL: */
   void *marg_cal_socket = scl_get_socket("marg_cal", "sub");
   THROW_IF(marg_cal_socket == NULL, -EIO);
   void *orientation_socket = scl_get_socket("orientation", "pub");
   THROW_IF(orientation_socket == NULL, -EIO);
   
   /* init logger: */
   THROW_ON_ERR(logger_open(name));

   /* init magnetic declination reader: */
   THROW_ON_ERR(scl_mag_decl_init());

   /* init opcd: */
   opcd_params_init(name, 1);
   
   /* init cal ahrs:*/
   cal_ahrs_init();
   
   /* init msgpack buffers: */
   msgpack_sbuffer *msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);
   msgpack_packer *pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   THROW_IF(pk == NULL, -ENOMEM);
 
   /* init auxiliary data structures: */
   marg_data_t marg_data;
   marg_data_init(&marg_data);
   interval_t interval;
   interval_init(&interval);

   ONCE(LOG(LL_INFO, "started decreasing beta gain"));
   while (running)
   {
      char buffer[1024];
      int ret = scl_recv_static(marg_cal_socket, buffer, sizeof(buffer));
      if (ret > 0)
      {
         msgpack_unpacked msg;
         msgpack_unpacked_init(&msg);
         if (msgpack_unpack_next(&msg, buffer, ret, NULL))
         {
            float dt = interval_measure(&interval);
            msgpack_object root = msg.data;
            assert (root.type == MSGPACK_OBJECT_ARRAY);
            uint8_t status = root.via.array.ptr[0].via.i64;
            FOR_N(i, 3)
               marg_data.gyro.ve[i] = root.via.array.ptr[1 + i].via.dec;
            FOR_N(i, 3)
               marg_data.acc.ve[i] = root.via.array.ptr[4 + i].via.dec;
            FOR_N(i, 3)
               marg_data.mag.ve[i] = root.via.array.ptr[7 + i].via.dec;
 
            euler_t euler;
            float declination = scl_mag_decl_get();
            int ahrs_state = cal_ahrs_update(&euler, &marg_data, declination, dt);
            if (ahrs_state == 0)
            {
               EVERY_N_TIMES(100, LOG(LL_INFO, "intermediate orientation (y p r): %f %f %f", 
                                      rad2deg(euler.yaw), rad2deg(euler.pitch), rad2deg(euler.roll)));
            }
            else if (ahrs_state == 1)
            {
               ONCE(LOG(LL_INFO, "final beta gain reached, final orientation (y p r): %.1f %.1f %.1f",
                        rad2deg(euler.yaw), rad2deg(euler.pitch), rad2deg(euler.roll)));
               EVERY_N_TIMES(10000, LOG(LL_INFO, "orientation (y p r): %f %f %f", 
                                       rad2deg(euler.yaw), rad2deg(euler.pitch), rad2deg(euler.roll)));
               msgpack_sbuffer_clear(msgpack_buf);
               msgpack_pack_array(pk, 3);
               PACKF(euler.yaw);
               PACKF(euler.pitch);
               PACKF(euler.roll);
               scl_copy_send_dynamic(orientation_socket, msgpack_buf->data, msgpack_buf->size);
            }
         }
         msgpack_unpacked_destroy(&msg);
      }
   }
 
   THROW_END();
}


void _cleanup(void)
{
   running = 0;
}


void main_wrap(int argc, char *argv[])
{
   (void)argc;
   (void)argv;
   int code = _main();
   if (code != 0)
   {
      LOG(LL_ERROR, "error: %d (%s)", code, strerror(-code));
      sleep(1);
   }
   exit(-code);   
}



int main(int argc, char *argv[])
{
   char pid_file[1024];
   service_name_to_pidfile(pid_file, name);
   daemonize(pid_file, main_wrap, _cleanup, argc, argv);
   return 0;
}

