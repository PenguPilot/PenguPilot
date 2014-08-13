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
  
 Remote Control Calibration Service Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <msgpack.h>
#include <daemon.h>
#include <util.h>
#include <scl.h>
#include <opcd_interface.h>
#include <serial.h>

#include "../shared/rc_cal.h"
#include "channels.h"


static int running = 1;
static msgpack_sbuffer *msgpack_buf = NULL;
static msgpack_packer *pk = NULL;
static char *platform = NULL;
static void *rc_socket = NULL;
static float channels[MAX_CHANNELS];
static void *scl_cal_socket = NULL;


int _main(void)
{
   THROW_BEGIN();

   /* initialize msgpack buffers: */
   msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);
   pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   THROW_IF(pk == NULL, -ENOMEM);
  
   /* initialize SCL: */
   THROW_ON_ERR(scl_init("rc_cal"));
   rc_socket = scl_get_socket("rc_raw");
   THROW_IF(rc_socket == NULL, -EIO);
   scl_cal_socket = scl_get_socket("rc_cal");
   THROW_IF(scl_cal_socket == NULL, -EIO);

   /* init opcd: */
   opcd_params_init("rc_cal", 0);
   
   /* init channel mapping: */
   THROW_ON_ERR(channels_init());
   
   while (1)
   {
      char buffer[1024];
      int ret = scl_recv_static(rc_socket, buffer, sizeof(buffer));
      if (ret > 0)
      {
         float channels[MAX_CHANNELS];
         msgpack_unpacked msg;
         msgpack_unpacked_init(&msg);
         if (msgpack_unpack_next(&msg, buffer, ret, NULL))
         {
            /* read received raw channels message: */
            msgpack_object root = msg.data;
            assert (root.type == MSGPACK_OBJECT_ARRAY);
            int n_channels = root.via.array.size - 1;
            int valid = root.via.array.ptr[0].via.i64;
            FOR_N(i, n_channels)
               if (i < MAX_CHANNELS)
                  channels[i] = root.via.array.ptr[1 + i].via.dec;
            
            /* apply permutation and calibration */
            float cal_channels[PP_MAX_CHANNELS];
            channels_update(cal_channels, channels);

            /* send the channels: */
            msgpack_sbuffer_clear(msgpack_buf);
            msgpack_pack_array(pk, PP_MAX_CHANNELS + 1);
            PACKI(valid);    /* index 0: valid */
            PACKFV(cal_channels, PP_MAX_CHANNELS); /* index 1, .. : channels */
            scl_copy_send_dynamic(scl_cal_socket, msgpack_buf->data, msgpack_buf->size);
         }
         msgpack_unpacked_destroy(&msg);
      }
      else
      {
         msleep(1);
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

   exit(-_main());   
}


int main(int argc, char *argv[])
{
   char pid_file[1024];
   sprintf(pid_file, "%s/.PenguPilot/run/rc_cal.pid", getenv("HOME"));
   daemonize(pid_file, main_wrap, _cleanup, argc, argv);
   return 0;
}

