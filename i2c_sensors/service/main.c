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
  
 Sensor Reader / Publisher

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <syslog.h>
#include <stdlib.h>
#include <daemon.h>
#include <stdbool.h>

#include <sched.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/mman.h>

#include <scl.h>
#include <opcd_interface.h>
#include <periodic_thread.h>
#include <util.h>
#include <interval.h>
#include <logger.h>

#include "platform/exynos_quad.h"
#include "platform/overo_quad.h"
#include "platform/pi_quad.h"

#define REALTIME_PERIOD 0.005


static struct sched_param sp;
static periodic_thread_t _thread; 
static periodic_thread_t *thread = &_thread;


static void _main(int argc, char *argv[])
{
   ASSERT_ONCE();

   /* init SCL subsystem: */
   syslog(LOG_INFO, "initializing signaling and communication link (SCL)");
   if (scl_init("10dof_sensor") != 0)
   {
      syslog(LOG_CRIT, "could not init scl module");
      die();
   }

   /* init params subsystem: */
   syslog(LOG_INFO, "initializing opcd interface");
   opcd_params_init("autopilot.", 1);

   /* initialize logger: */
   syslog(LOG_INFO, "opening logger");
   if (logger_open("autopilot") != 0)
   {
      syslog(LOG_CRIT, "could not open logger");
      die();
   }
   syslog(LOG_CRIT, "logger opened");

   LOG(LL_INFO, "initializing platform");

   char *plat_name = NULL;
   opcd_param_get("platform", &plat_name);
   if (strcmp(plat_name, "overo_quad") == 0)
   {
      if (overo_quad_init(&platform) < 0)
      {
         LOG(LL_ERROR, "could not initialize platform");
      }
   }
   else if (strcmp(plat_name, "pi_quad") == 0)
   {
      if (pi_quad_init(&platform) < 0)
      {
         LOG(LL_ERROR, "could not initialize platform");
      }
   }
   else if (strcmp(plat_name, "exynos_quad") == 0)
   {
      if (exynos_quad_init(&platform) < 0)
      {
         LOG(LL_ERROR, "could not initialize platform");
      }
   }
   else
   {
      LOG(LL_ERROR, "unknown platform: %s", plat_name);
   }


   LOG(LL_INFO, "setting up real-time scheduling");
   sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
   sched_setscheduler(getpid(), SCHED_FIFO, &sp);
   
   if (nice(-20) == -1)
   {
      LOG(LL_ERROR, "could not renice process");
   }
   
   thread->sched_param.sched_priority = 97;
   pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread->sched_param);

   thread->name = "main_loop";
   thread->running = 1;
   thread->periodic_data.period.tv_sec = 0;
   thread->periodic_data.period.tv_nsec = NSEC_PER_SEC * REALTIME_PERIOD;

   interval_t interval;
   interval_init(&interval);

   PERIODIC_THREAD_LOOP_BEGIN
   {
      marg_data_t marg_data;
      float ultra_z;
      float baro_z;
      float dt = interval_measure(&interval);
      uint8_t status = platform_read_sensors(&marg_data, &ultra_z, &baro_z);
   }
   PERIODIC_THREAD_LOOP_END
}


int main(int argc, char *argv[])
{
   _main(argc, argv);
   char pid_file[1024];
   sprintf(pid_file, "%s/.PenguPilot/run/sensors_reader.pid", getenv("HOME"));
   daemonize(pid_file, _main, die, argc, argv);
   return 0;
}


