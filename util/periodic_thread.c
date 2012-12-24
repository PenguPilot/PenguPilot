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

/*
 * periodic_thread.h
 *
 * periodic, fixed-priority threads
 *
 *
 *  Created on: 15.06.2010
 *      Author: tobi
 */


#include <limits.h>
#include <stdio.h>


#include "util.h"
#include "periodic_thread.h"
#include "thread_config.h"


void periodic_thread_start(periodic_thread_t *thread, void *(*func)(void *),
                           const char *name, int priority, struct timespec period, void *private)
{
   ASSERT_NOT_NULL(thread);
   ASSERT_NOT_NULL(func);
   ASSERT_NOT_NULL(name);
   ASSERT_FALSE(thread->running);

   thread->running = 1;
   thread->name = name;
   thread->private = private;
   thread->periodic_data.period = period;
   (void)pthread_attr_init(&thread->attr);
   (void)pthread_attr_setschedpolicy(&thread->attr, SCHED_FIFO);
   thread->sched_param.sched_priority = priority;
   (void)pthread_attr_setschedparam(&thread->attr, &thread->sched_param);
   (void)pthread_attr_setstacksize(&thread->attr, PTHREAD_STACK_MIN + THREAD_STACK_SIZE);
   (void)pthread_create(&thread->handle, &thread->attr, func, thread);
}


void periodic_thread_stop(periodic_thread_t *thread)
{
   ASSERT_NOT_NULL(thread);
   ASSERT_TRUE(thread->running);

   thread->running = 0;
   (void)pthread_join(thread->handle, NULL);
}


void periodic_thread_init_period(periodic_thread_t *thread)
{
   ASSERT_NOT_NULL(thread);
   ASSERT_TRUE(thread->running);

   (void)clock_gettime(CLOCK_MONOTONIC, &thread->periodic_data.next);
}


int periodic_thread_wait_for_next_period(periodic_thread_t *thread)
{
   ASSERT_NOT_NULL(thread);
   ASSERT_TRUE(thread->running);

   int ret;
   struct timespec ts_result;
   (void)clock_gettime(CLOCK_MONOTONIC, &thread->periodic_data.now);
   TIMESPEC_ADD(ts_result, thread->periodic_data.next, thread->periodic_data.period);
   thread->periodic_data.next = ts_result;
   (void)clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &thread->periodic_data.next, NULL);
   if (timespec_cmp(&thread->periodic_data.now, &thread->periodic_data.next) < 0)
   {
      ret = 0;
   }
   else
   {
      /*printf("thread %s missed deadline. now: %ld sec %ld nsec next: %ld sec %ld nsec \n",
          thread->name, thread->periodic_data.now.tv_sec, thread->periodic_data.now.tv_nsec,
          thread->periodic_data.next.tv_sec, thread->periodic_data.next.tv_nsec);
      */
      ret = 1;
   }
   return ret;
}
