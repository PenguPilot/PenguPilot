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
 
 Copyright (C) 2010 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <stdio.h>
#include <limits.h>

#include "util.h"
#include "simple_thread.h"
#include "thread_config.h"


void simple_thread_start(simple_thread_t *thread, void *(*func)(void *),
                         char *name, int priority, void *private)
{
   ASSERT_NOT_NULL(thread);
   ASSERT_NOT_NULL(func);
   ASSERT_NOT_NULL(name);
   ASSERT_FALSE(thread->running);

   thread->running = 1;
   thread->name = name;
   thread->private = private;
   (void)pthread_attr_init(&thread->attr);
   (void)pthread_attr_setschedpolicy(&thread->attr, SCHED_FIFO);
   thread->sched_param.sched_priority = priority;
   (void)pthread_attr_setschedparam(&thread->attr, &thread->sched_param);
   (void)pthread_attr_setstacksize(&thread->attr, PTHREAD_STACK_MIN + THREAD_STACK_SIZE);
   (void)pthread_create(&thread->handle, &thread->attr, func, thread);
}


void simple_thread_stop(simple_thread_t *thread)
{
   ASSERT_NOT_NULL(thread);
   ASSERT_TRUE(thread->running);

   thread->running = 0;
   (void)pthread_join(thread->handle, NULL);
}
