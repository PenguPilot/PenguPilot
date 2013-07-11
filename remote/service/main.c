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
  
 Real-Time Main Implementation

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

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
#include <sclhelper.h>
#include <daemon.h>
#include "rc_dsl/rc_dsl_reader.h"
#include <rc.pb-c.h>
#include <syslog.h>


static struct sched_param sp;
static int running = 1;
static void *rc_socket = NULL;


static int realtime_init(void)
{
   ASSERT_ONCE();

   if (system("echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor") != 0)
   {
      return -1;
   }
 
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


void _main(int argc, char *argv[])
{
   (void)argc;
   (void)argv;
  
   while (running)
   {
      float channels[64];
      float rssi = rc_dsl_reader_get(channels);
      printf("rssi: %f\n", rssi);
      RCData rc_data = RCDATA__INIT;
      rc_data.pitch = channels[0];
      rc_data.roll = channels[1];
      rc_data.yaw = channels[2];
      rc_data.gas = channels[3];
      rc_data.switch1 = channels[4];
      rc_data.has_switch1 = 1;
      SCL_PACK_AND_SEND_DYNAMIC(rc_socket, rcdata, rc_data);
      msleep(50);
   }
}


void _cleanup(void)
{
   running = 0;
}


int main(int argc, char *argv[])
{
   if (realtime_init() < 0)
   {
      return EXIT_FAILURE;
   }

   if (rc_dsl_reader_init() < 0)
   {
      return EXIT_FAILURE;
   }
   
   if (scl_init("rc") != 0)
   {
      syslog(LOG_CRIT, "could not init scl module");
      return EXIT_FAILURE;
   }

   rc_socket = scl_get_socket("data");
   if (rc_socket == NULL)
   {
      syslog(LOG_CRIT, "could not get scl gate");
      return EXIT_FAILURE;
   }
 
   daemonize("/var/run/rc.pid", _main, _cleanup, argc, argv);
   return 0;
}

