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
  
 SCL GPS Data Reader Implementation

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <util.h>
#include <simple_thread.h>
#include <gps_data.pb-c.h>
#include <scl.h>
#include "../../../util/time/interval.h"

#include "scl_gps.h"


#define THREAD_PRIORITY 98



/* if we receive no valid GPS fix for more than this amount of time,
   indicate an error;
   this condition applies if:
     - the gps service does not send data (driver or bus problem)
     - the gps service sends data but the fix is not 2D/3D */

#define GPS_TIMEOUT (5.0f)



static gps_data_t gps_data = {FIX_NOT_SEEN, 0, 0, 0, 0};
static simple_thread_t thread;
static void *scl_socket;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static interval_t interval;
static float timer = 0.0f;


SIMPLE_THREAD_BEGIN(thread_func)
{
   SIMPLE_THREAD_LOOP_BEGIN
   {
      GpsData *_gps_data;
      SCL_RECV_AND_UNPACK_DYNAMIC(_gps_data, scl_socket, gps_data);
      if (_gps_data != NULL)
      {
         if (_gps_data->fix >= 2)
         {
            pthread_mutex_lock(&mutex);
            timer = 0.0f; /* reset timer */
            gps_data.fix = _gps_data->fix;
            gps_data.sats = _gps_data->sats;
            gps_data.lat = _gps_data->lat;
            gps_data.lon = _gps_data->lon;
            gps_data.alt = _gps_data->alt;
            pthread_mutex_unlock(&mutex);
         }
         SCL_FREE(gps_data, _gps_data);
      }
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END


int scl_gps_init(void)
{
   ASSERT_ONCE();
   scl_socket = scl_get_socket("gps");
   ASSERT_NOT_NULL(scl_socket);
   simple_thread_start(&thread, thread_func, "gps_reader", THREAD_PRIORITY, NULL);
   interval_init(&interval);
   return 0;
}


int scl_gps_read(gps_data_t *data_out)
{
   ASSERT_NOT_NULL(data_out);
   int ret_code = 0;
   pthread_mutex_lock(&mutex);
   timer += interval_measure(&interval);
   if (timer > GPS_TIMEOUT)
   {
      ret_code = -1;
   }
   else
   {
      *data_out = gps_data;
   }
   pthread_mutex_unlock(&mutex);
   return ret_code;
}

