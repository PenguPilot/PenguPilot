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
  
 Simple Threads Interface
 
 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __SIMPLE_THREAD_H__
#define __SIMPLE_THREAD_H__


#include <pthread.h>

#include "thread_realtime.h"


typedef struct
{
   char *name;
   pthread_t handle;
   volatile int running;
   pthread_attr_t attr;
   struct sched_param sched_param;
   void *private;
}
simple_thread_t;


#define SIMPLE_THREAD_BEGIN(name) \
   static void *name(void *__arg) \
   { \
      simple_thread_t *thread = (simple_thread_t *)__arg;

#define SIMPLE_THREAD_LOOP_BEGIN \
   thread_stack_prefault(); \
   while (thread->running) \
   {

#define SIMPLE_THREAD_LOOP_END \
   }

#define SIMPLE_THREAD_END \
   return NULL; }


void simple_thread_start(simple_thread_t *thread, void *(*func)(void *),
                         char *name, int priority, void *private);

void simple_thread_stop(simple_thread_t *thread);


#endif /* __SIMPLE_THREAD_H__ */

