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
  
 Remote Control Service Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include <sched.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <msgpack.h>

#include <daemon.h>
#include <util.h>
#include <scl.h>
#include <opcd_interface.h>
#include <serial.h>

#include "rc_dsl.h"


static struct sched_param sp;
static int running = 1;


static int realtime_init(void)
{
   ASSERT_ONCE();

   sp.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
   sched_setscheduler(getpid(), SCHED_FIFO, &sp);

   if (nice(-20) == -1)
   {
      return -2;
   }

   if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
   {
      return -3;
   }
   return 0;
}




int _main(void)
{
   THROW_BEGIN();
   THROW_ON_ERR(realtime_init());
   
   THROW_ON_ERR(scl_init("remote"));
   opcd_params_init("", 0);
   
   char *platform = NULL;
   opcd_param_get("platform", &platform);
   char buffer[128];
   strcpy(buffer, platform);
   strcat(buffer, ".dsl_serial_path");
   char *dev_path;
   opcd_param_get(buffer, &dev_path);
   serialport_t port;
   THROW_ON_ERR(serial_open(&port, dev_path, 38400, 0, 0, 0));
   rc_dsl_t rc_dsl;
   rc_dsl_init(&rc_dsl);

   void *rc_socket = scl_get_socket("remote");
   THROW_IF(rc_socket == NULL, -1);
   
   msgpack_sbuffer *msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(!msgpack_buf, -ENOMEM);
   msgpack_packer *pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   THROW_IF(!pk, -ENOMEM);
 
   while (running)
   {
      int b = serial_read_char(&port);
      if (b < 0)
      {
         usleep(1000);
         continue;
      }
      int status = rc_dsl_parse_dsl_data(&rc_dsl, (uint8_t)b);
      if (status == 1)
      {
         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, RC_DSL_CHANNELS + 1);
         PACKI(RC_DSL_RSSI_VALID(rc_dsl.RSSI)); /* valid */
         PACKFV(rc_dsl.channels, RC_DSL_CHANNELS); /* channels */
         scl_copy_send_dynamic(rc_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
   THROW_END();
}


void _cleanup(void)
{
   running = 0;
}


main_wrap(int argc, char *argv[])
{
   (void)argc;
   (void)argv;

   exit(-_main());   
}


int main(int argc, char *argv[])
{
   char pid_file[1024];
   sprintf(pid_file, "%s/.PenguPilot/run/remote.pid", getenv("HOME"));
   daemonize(pid_file, main_wrap, _cleanup, argc, argv);
   return 0;
}

