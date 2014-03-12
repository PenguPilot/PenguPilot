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
  
 I2CXL Reader Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include <serial.h>
#include <simple_thread.h>
#include <threadsafe_types.h>

#include "i2cxl_reader.h"
#include "i2cxl.h"


#define THREAD_NAME       "i2cxl_reader"
#define THREAD_PRIORITY   98


static simple_thread_t thread;
static i2cxl_t i2cxl;
static tsfloat_t altitude;
static int status;


SIMPLE_THREAD_BEGIN(thread_func)
{
   SIMPLE_THREAD_LOOP_BEGIN
   {
      float alt;
      status = i2cxl_read(&i2cxl, &alt);
      if (status == 0)
      {
         tsfloat_set(&altitude, alt);
      }
      msleep(50);
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END


int i2cxl_reader_init(i2c_bus_t *bus)
{
   ASSERT_ONCE();
   tsfloat_init(&altitude, 0.2);
   i2cxl_init(&i2cxl, bus);
   simple_thread_start(&thread, thread_func, THREAD_NAME, THREAD_PRIORITY, NULL);
   return 0;
}


int i2cxl_reader_get_alt(float *alt)
{
   if (status == 0)
   {
      *alt = tsfloat_get(&altitude);
   }
   return status;
}

