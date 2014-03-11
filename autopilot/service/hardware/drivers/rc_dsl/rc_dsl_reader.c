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
  
 RC DSL Reader Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <string.h>


#include <util.h>
#include <serial.h>
#include <simple_thread.h>
#include <opcd_interface.h>
#include <pthread.h>


#include "rc_dsl_reader.h"
#include "../../../util/logger/logger.h"


#define THREAD_NAME       "rc_dsl_reader"
#define THREAD_PRIORITY   98


#define RSSI_MIN RSSI_SCALE(7)


static simple_thread_t thread;
static serialport_t port;
static rc_dsl_t rc_dsl;
static char *dev_path = NULL;
static pthread_mutexattr_t mutexattr;   
static pthread_mutex_t mutex;
static float channels[RC_DSL_CHANNELS];
static int sig_valid = 0;


SIMPLE_THREAD_BEGIN(thread_func)
{
   SIMPLE_THREAD_LOOP_BEGIN
   {
      int b = serial_read_char(&port);
      if (b < 0)
      {
         msleep(1);
      }
      int status = rc_dsl_parse_dsl_data(&rc_dsl, (uint8_t)b);
      if (status < 0)
      {
         LOG(LL_ERROR, "could not parse dsl frame");   
      }
      else if (status == 1)
      {
         pthread_mutex_lock(&mutex);
         sig_valid = rc_dsl.RSSI > RSSI_MIN;
         memcpy(channels, rc_dsl.channels, sizeof(channels));   
         pthread_mutex_unlock(&mutex);
      }
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END



int rc_dsl_reader_init(void)
{
   ASSERT_ONCE();
   THROW_BEGIN();
   memset(channels, 0, sizeof(channels));
   opcd_param_t params[] =
   {
      {"serial_port", &dev_path},
      OPCD_PARAMS_END   
   };
   opcd_params_apply("sensors.rc_dsl.", params);
   THROW_ON_ERR(serial_open(&port, dev_path, 38400, 0, 0, 0));
   rc_dsl_init(&rc_dsl);
   pthread_mutexattr_init(&mutexattr); 
   pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT); 
   pthread_mutex_init(&mutex, &mutexattr);
   simple_thread_start(&thread, thread_func, THREAD_NAME, THREAD_PRIORITY, NULL);

   THROW_END();
}


int rc_dsl_reader_get(float channels_out[RC_DSL_CHANNELS])
{
   pthread_mutex_lock(&mutex);
   memcpy(channels_out, channels, sizeof(channels));
   int ret = sig_valid ? 0 : -EAGAIN;
   pthread_mutex_unlock(&mutex);
   return ret;
}

