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
  
 SCL Power Reader Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include <unistd.h>

#include <util.h>
#include <simple_thread.h>
#include <power.pb-c.h>
#include <scl.h>

#include "scl_power.h"
#include "../../../filters/filter.h"
#include "../../../util/logger/logger.h"


#define THREAD_PRIORITY 98


static simple_thread_t thread;
static pthread_mutexattr_t mutexattr;
static pthread_mutex_t mutex;
static void *socket;
static float voltage = 16.0;
static float current = 0.4;


static void scl_read_power(float *_voltage, float *_current)
{
   PowerState *state;
   SCL_RECV_AND_UNPACK_DYNAMIC(state, socket, power_state);
   if (state)
   {
      *_voltage = state->voltage;
      *_current = state->current;
      SCL_FREE(power_state, state);
   }
   else
   {
      sleep(1);
      LOG(LL_ERROR, "could not read voltage");
   }
}


SIMPLE_THREAD_BEGIN(thread_func)
{
   /* initial measurement: */
   scl_read_power(&voltage, &current);
   
   /* filter set-up: */
   Filter1 voltage_filter;
   filter1_lp_init(&voltage_filter, 0.1f, 1.0f, 1);
   voltage_filter.z[0] = voltage;
   Filter1 current_filter;
   filter1_lp_init(&current_filter, 0.1f, 1.0f, 1);
   current_filter.z[0] = current;

   /* power state reading loop: */
   SIMPLE_THREAD_LOOP_BEGIN
   {
      float voltage_raw;
      float current_raw;
      scl_read_power(&voltage_raw, &current_raw);
      
      pthread_mutex_lock(&mutex);
      filter1_run(&voltage_filter, &voltage_raw, &voltage);
      filter1_run(&current_filter, &current_raw, &current);
      pthread_mutex_unlock(&mutex);
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END


int scl_power_init(void)
{
   socket = scl_get_socket("power");
   if (socket == NULL)
   {
      return -1;
   }
   pthread_mutexattr_init(&mutexattr);
   pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT);
   pthread_mutex_init(&mutex, &mutexattr);
   simple_thread_start(&thread, thread_func, "voltage_reader", THREAD_PRIORITY, NULL);
   return 0;
}


int scl_power_read(float *voltage_out, float *current_out)
{
   pthread_mutex_lock(&mutex);
   *voltage_out = voltage;
   *current_out = current;
   pthread_mutex_unlock(&mutex);
   return 0;
}

