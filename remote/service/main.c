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
#include <util.h>
#include <scl.h>
#include <daemon.h>
#include "rc_dsl/rc_dsl_reader.h"
#include <msgpack.h>
#include <syslog.h>


static struct sched_param sp;
static int running = 1;


static int realtime_init(void)
{
   ASSERT_ONCE();

   sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
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


static msgpack_sbuffer *msgpack_buf = NULL;
static msgpack_packer *pk = NULL;


void _main(int argc, char *argv[])
{
   (void)argc;
   (void)argv;
   
   if (realtime_init() < 0)
   {
      exit(EXIT_FAILURE);
   }

   if (scl_init("remote") != 0)
   {
      syslog(LOG_CRIT, "could not init scl module");
      exit(EXIT_FAILURE);
   }

   if (rc_dsl_reader_init() < 0)
   {
      exit(EXIT_FAILURE);
   }
   
   void *rc_socket = scl_get_socket("remote");
   if (rc_socket == NULL)
   {
      syslog(LOG_CRIT, "could not get scl gate");
      exit(EXIT_FAILURE);
   }
   
   ASSERT_NULL(msgpack_buf);
   msgpack_buf = msgpack_sbuffer_new();
   ASSERT_NOT_NULL(msgpack_buf);
   ASSERT_NULL(pk);
   pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   ASSERT_NOT_NULL(pk);
 
   while (running)
   {
      float rssi;
      float channels[16];
      int status = rc_dsl_reader_get(&rssi, channels);
      if (status == 0)
      {
         msgpack_pack_array(pk, 16);
         PACKF(rssi);
         PACKFV(channels, 16);
         scl_copy_send_dynamic(rc_socket, msgpack_buf->data, msgpack_buf->size);
         msgpack_sbuffer_clear(msgpack_buf);
      }
      usleep(1000 * 10);
   }
}


void _cleanup(void)
{
   running = 0;
}


int main(int argc, char *argv[])
{
   char pid_file[1024];
   sprintf(pid_file, "%s/.PenguPilot/run/remote.pid", getenv("HOME"));
   daemonize(pid_file, _main, _cleanup, argc, argv);
   return 0;
}

