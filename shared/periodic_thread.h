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

 Periodic Fixed-Priority Threads Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#ifndef __PERIODIC_THREAD_H__
#define __PERIODIC_THREAD_H__


#include <time.h>
#include <pthread.h>


#include "periodic_thread.h"
#include "thread_realtime.h"


typedef struct
{
   struct timespec now;
   struct timespec next;
   struct timespec period;
}
period_t;


typedef struct
{
   const char *name;
   period_t periodic_data;
   pthread_t handle;
   volatile int running;
   pthread_attr_t attr;
   struct sched_param sched_param;
   void *private;
}
periodic_thread_t;


#define PERIODIC_THREAD_BEGIN(name) \
   static void *name(void *__arg) \
   { \
      periodic_thread_t *thread = (periodic_thread_t *)__arg; \
       
#define PERIODIC_THREAD_LOOP_BEGIN \
   thread_stack_prefault(); \
   periodic_thread_init_period(thread); \
   while (thread->running) \
   { \
      periodic_thread_wait_for_next_period(thread);

#define PERIODIC_THREAD_LOOP_END \
   }

#define PERIODIC_THREAD_END \
   return NULL; }


void periodic_thread_start(periodic_thread_t *thread, void *(*func)(void *),
                           const char *name, int priority, struct timespec period, void *private);

void periodic_thread_stop(periodic_thread_t *thread);


void periodic_thread_init_period(periodic_thread_t *thread);


int periodic_thread_wait_for_next_period(periodic_thread_t *thread);


#endif /* __PERIODIC_THREAD_H__ */

