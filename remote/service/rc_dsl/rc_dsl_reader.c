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



#include <pthread.h>
#include <string.h>


#include <util.h>
#include <serial.h>
#include <simple_thread.h>
#include <opcd_interface.h>


#include "rc_dsl_reader.h"


#define THREAD_NAME       "rc_dsl_reader"
#define THREAD_PRIORITY   98


static simple_thread_t thread;
static serialport_t port;
static rc_dsl_t rc_dsl;
static char *dev_path = NULL;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static int status = -1;
static float signal_out = 0.0f;
static float channels[RC_DSL_CHANNELS];


SIMPLE_THREAD_BEGIN(thread_func)
{
   SIMPLE_THREAD_LOOP_BEGIN
   {
      int b = serial_read_char(&port);
      pthread_mutex_lock(&mutex);
      if (b < 0)
      {
         status = -1; /* something went wrong; immediately signal this condition */
         msleep(1);
      }
      int _status = rc_dsl_parse_dsl_data(&rc_dsl, (uint8_t)b);
      if (_status == 1)
      {
         memcpy(channels, rc_dsl.channels, sizeof(channels));   
         signal_out = rc_dsl.RSSI;
         status = 0;
      }
      else if (_status < 0)
      {
         status = -1;
      }
      pthread_mutex_unlock(&mutex);
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END



int rc_dsl_reader_init(void)
{
   ASSERT_ONCE();
   opcd_params_init("", 0);
   ASSERT_ONCE();
   THROW_BEGIN();
   memset(channels, 0, sizeof(channels));
   char *platform = NULL;
   opcd_param_get("platform", &platform);
   char buffer[128];
   strcpy(buffer, platform);
   free(platform);
   strcat(buffer, ".dsl_serial_path");
   opcd_param_get(buffer, &dev_path);
   THROW_ON_ERR(serial_open(&port, dev_path, 38400, 0, 0, 0));
   rc_dsl_init(&rc_dsl);
   simple_thread_start(&thread, thread_func, THREAD_NAME, THREAD_PRIORITY, NULL);
   THROW_END();
}


int rc_dsl_reader_get(float *rssi, float channels_out[RC_DSL_CHANNELS])
{
   pthread_mutex_lock(&mutex);
   memcpy(channels_out, channels, sizeof(channels));
   *rssi = signal_out;
   int _status = status;
   pthread_mutex_unlock(&mutex);
   return _status;
}

