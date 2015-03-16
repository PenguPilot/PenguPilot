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
  
 Motors Service

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <pthread.h>
#include <syslog.h>
#include <msgpack.h>
#include <stdlib.h>
#include <sys/mman.h>

#include <daemon.h>
#include <util.h>
#include <scl.h>
#include <logger.h>
#include <sched.h>
#include <opcd_interface.h>

#include "force_to_esc/force_to_esc.h"


static struct sched_param sp;
static bool running = true;
static msgpack_sbuffer *msgpack_buf = NULL;
static char *platform = NULL;
static void *forces_socket = NULL;


int __main(void)
{
   THROW_BEGIN();

   /* initialize SCL: */
   forces_socket = scl_get_socket("motor_forces", "sub");
   THROW_IF(forces_socket == NULL, -EIO);

   /* init opcd: */
   opcd_params_init("", 0);
   char *platform;
   opcd_param_get("platform", &platform);
   syslog(LOG_INFO, "platform: %s", platform);
   
   /* initialize logger: */
   syslog(LOG_INFO, "opening logger");
   if (logger_open("motors") != 0)
   {  
      syslog(LOG_CRIT, "could not open logger");
      THROW(-EIO);
   }
   syslog(LOG_INFO, "logger opened");

   /* real-time stuff: */
   LOG(LL_INFO, "setting up real-time scheduling");
   sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
   sched_setscheduler(getpid(), SCHED_FIFO, &sp);
 
   if (nice(-20) == -1)
   {
      LOG(LL_ERROR, "could not renice process");
   }
 
   /* initialize msgpack buffers: */
   msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);

   /* determine motor f2e: */
   char buffer_path[128];
   strcpy(buffer_path, platform);
   strcat(buffer_path, ".motors.f2e");
   char *f2e_name;
   opcd_param_get(buffer_path, &f2e_name);
   LOG(LL_INFO, "f2e: %s", f2e_name);
   float (*f2e)(float, float);
   if (strcmp(f2e_name, "mk12_roxxy282735_1045") == 0)
      f2e = f2e_mk12_roxxy282735_1045;
   else if (strcmp(f2e_name, "hk20_roxxy282735_1045") == 0)
      f2e = f2e_hk20_roxxy282735_1045;   
   else if (strcmp(f2e_name, "hexfet20_suppo221213_1045") == 0)
      f2e = f2e_hexfet20_suppo221213_1045;   
   else
      THROW(-EINVAL);
 
   /* determine number of motors: */
   strcpy(buffer_path, platform);
   strcat(buffer_path, ".motors.n_motors");
   int n_motors;
   opcd_param_get(buffer_path, &n_motors);
   LOG(LL_INFO, "number of motors: %d", n_motors);

   /* determine motor driver: */
   strcpy(buffer_path, platform);
   strcat(buffer_path, ".motors.driver");
   char *driver;
   opcd_param_get(buffer_path, &driver);
   LOG(LL_INFO, "driver: %s", driver);


 
   while (running)
   {
      char buffer[1024];
      int ret = scl_recv_static(forces_socket, buffer, sizeof(buffer));
      if (ret > 0)
      {
         msgpack_unpacked msg;
         msgpack_unpacked_init(&msg);
         if (msgpack_unpack_next(&msg, buffer, ret, NULL))
         {
            /* read received raw channels message: */
            msgpack_object root = msg.data;
            assert (root.type == MSGPACK_OBJECT_ARRAY);
            int n_forces = root.via.array.size;
            FOR_N(i, n_forces)
            {
               float force = root.via.array.ptr[i].via.dec;
            }
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


void _main(int argc, char *argv[])
{
   __main();
}


int main(int argc, char *argv[])
{
   __main();
   char pid_file[1024];
   sprintf(pid_file, "%s/.PenguPilot/run/motors.pid", getenv("HOME"));
   daemonize(pid_file, _main, NULL, argc, argv);
   return 0;
}

