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
  
 File Purpose

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

#include <periodic_thread.h>
#include <util.h>

#include "main_util.h"
#include "main_loop.h"
#include "../util/logger/logger.h"
#include "../util/time/interval.h"


static struct sched_param sp;
static periodic_thread_t _thread; 
static periodic_thread_t *thread = &_thread;


static void main_realtime_init(void)
{
   ASSERT_ONCE();

   LOG(LL_INFO, "setting maximum CPU clock");
   if (system("echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor") != 0)
   {
      LOG(LL_ERROR, "failed");
      die();
   }
 
   LOG(LL_INFO, "setting up real-time scheduling");
   sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
   sched_setscheduler(getpid(), SCHED_FIFO, &sp);

   if (nice(-20) == -1)
   {
      LOG(LL_ERROR, "could not renice process");
      die();
   }

   if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
   {
      LOG(LL_ERROR, "mlockall() failed");
      die();
   }

   thread->name = "main_loop";
   thread->running = 1;
   thread->periodic_data.period.tv_sec = 0;
   thread->periodic_data.period.tv_nsec = NSEC_PER_SEC * REALTIME_PERIOD;
}


/* executed on the actual hardware */
void main_realtime(int argc, char *argv[])
{
   (void)argc;
   (void)argv;
   main_realtime_init();
   main_init(0);
   interval_t interval;
   interval_init(&interval);
   DATA_DEFINITION();
   PERIODIC_THREAD_LOOP_BEGIN
   {
      dt = interval_measure(&interval);
      uint16_t sensor_status = platform_read_sensors(&marg_data, &gps_data, &ultra_z, &baro_z, &voltage, channels);
      main_step(dt, &marg_data, &gps_data, ultra_z, baro_z, voltage, channels, sensor_status, 0);
   }
   PERIODIC_THREAD_LOOP_END
}

