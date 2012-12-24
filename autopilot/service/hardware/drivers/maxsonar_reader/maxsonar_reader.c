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




#include <util.h>
#include <serial.h>
#include <simple_thread.h>
#include <threadsafe_types.h>

#include "maxsonar_reader.h"
#include "../maxsonar/maxsonar.h"
#include "../../../filters/median_filter.h"


#define THREAD_NAME       "maxsonar_reader"
#define THREAD_PRIORITY   0

#define USE_FILTER


static simple_thread_t thread;


static serialport_t port;
static maxsonar_t *sonar = NULL;

static tsfloat_t altitude;


SIMPLE_THREAD_BEGIN(thread_func)
{
#ifdef USE_FILTER
   median_filter_t filter;
   median_filter_init(&filter, 5);
#endif
   SIMPLE_THREAD_LOOP_BEGIN
   {
      uint8_t b = serial_read_char(&port);
      int status = maxsonar_parse(sonar, b);
      if (status == 1)
      {
#ifdef USE_FILTER
         tsfloat_set(&altitude, median_filter_run(&filter, maxsonar_get_dist(sonar)));
#else
         tsfloat_set(&altitude, maxsonar_get_dist(sonar));
#endif
      }
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END


int maxsonar_reader_init(void)
{
   ASSERT_ONCE();
   int status = serial_open(&port, "/dev/ttyO0", 9600, 0, 0, 0);
   if (status != 0)
   {
      return status;   
   }
   tsfloat_init(&altitude, 0.3);
   sonar = maxsonar_create();
   simple_thread_start(&thread, thread_func, THREAD_NAME, THREAD_PRIORITY, NULL);
   return 0;
}


float maxsonar_reader_get_alt(void)
{
   return tsfloat_get(&altitude);
}

