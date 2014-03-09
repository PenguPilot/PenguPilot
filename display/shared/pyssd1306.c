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
 
 SSD1306 Python Implementation Wrapper for SWIG

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

#include "ssd1306.h"


#define ASSERT_INIT() if (!initialized) return


static i2c_bus_t i2c_bus;
static i2c_dev_t i2c_dev;
static ssd1306_t ssd;
static bool initialized = false;


void init(char *_i2c_bus)
{
   assert(_i2c_bus);
   if (initialized)
      return;
   i2c_bus_open(&i2c_bus, _i2c_bus);
   i2c_dev_init(&i2c_dev, &i2c_bus, 0x3d);
   ssd1306_init(&ssd, &i2c_dev);
   initialized = true;
}


void blit(char *data)
{
   for (int x = 0; x < ssd.width; x++)
      for (int y = 0; y < ssd.height; y++)
         ssd1306_set_pixel(&ssd, x, y, data[x + y * ssd.width]);   
   ssd1306_update(&ssd);
}


void set_pixel(int16_t x, int16_t y, uint16_t color)
{
   ASSERT_INIT();
   ssd1306_set_pixel(&ssd, x, y, color);   
}


void invert(bool i)
{
   ASSERT_INIT();
   ssd1306_invert(&ssd, i);   
}


void clear(void)
{
   ASSERT_INIT();
   ssd1306_clear(&ssd);
}


void update(void)
{
   ASSERT_INIT();
   ssd1306_update(&ssd);
}

