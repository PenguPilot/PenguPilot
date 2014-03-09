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
  
 MS5611 Reader Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

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

#include "ms5611_reader.h"
#include "ms5611.h"


#define THREAD_NAME       "ms5611_reader"
#define THREAD_PRIORITY   98


static simple_thread_t thread;
static ms5611_t ms5611;
static tsfloat_t altitude;
static int status;


SIMPLE_THREAD_BEGIN(thread_func)
{
   SIMPLE_THREAD_LOOP_BEGIN
   {
      status = ms5611_measure(&ms5611);
      if (status == 0)
      {
         tsfloat_set(&altitude, ms5611.c_a);
      }
      msleep(20);
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END


int ms5611_reader_init(i2c_bus_t *bus)
{
   ASSERT_ONCE();
   tsfloat_init(&altitude, 0.0);
   ms5611_init(&ms5611, bus, MS5611_OSR4096, MS5611_OSR4096);
   simple_thread_start(&thread, thread_func, THREAD_NAME, THREAD_PRIORITY, NULL);
   return 0;
}


int ms5611_reader_get_alt(float *alt)
{
   if (status == 0)
   {
      *alt = tsfloat_get(&altitude);
   }
   return status;
}

