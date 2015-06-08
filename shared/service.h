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
  
 Simple Service Helper Interface

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <syslog.h>
#include <sched.h>
#include <unistd.h>
#include <stdbool.h>

#include <opcd_interface.h>
#include <daemon.h>
#include <logger.h>
#include <util.h>

#define SERVICE_MAIN_DEBUG false

#define SERVICE_MAIN_BEGIN(name, prio) \
   \
   static char *service_name = name; \
   static bool running = true; \
   static bool debug = SERVICE_MAIN_DEBUG; \
   \
   static int _main(void) \
   { \
   THROW_BEGIN(); \
   \
   /* set up real-time scheduling: */ \
   struct sched_param __sp; \
   __sp.sched_priority = prio; \
   sched_setscheduler(getpid(), SCHED_FIFO, &__sp); \
   \
   /* initialize logger: */ \
   syslog(LOG_INFO, "opening logger"); \
   if (logger_open(name) != 0) \
   {  \
      syslog(LOG_CRIT, "could not open logger"); \
      THROW(-EIO); \
   } \
   LOG(LL_INFO, "starting service: %s", name); \
   opcd_params_init(name, true);


#define SERVICE_MAIN_PSEUDO_LOOP() \
   while (1) \
      sleep(10000);

#define SERVICE_MAIN_END \
      THROW_END(); \
   } \
   \
   static void _cleanup(void) \
   { \
      running = false; \
   } \
   \
   static void main_wrap(int argc, char *argv[]) \
   { \
      (void)argc; \
      (void)argv; \
      \
      exit(-_main()); \
   } \
   \
   int main(int argc, char *argv[]) \
   { \
      if (debug) \
         main_wrap(argc, argv); \
      char pid_file[1024]; \
      service_name_to_pidfile(pid_file, service_name); \
      daemonize(pid_file, main_wrap, _cleanup, argc, argv); \
      return 0; \
   }

