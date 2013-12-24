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
  
 Mikrokopter Brushless Motor Driver Implementation

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <malloc.h>
#include <math.h>

#include <util.h>

#include "holger_blmc.h"


static i2c_dev_t *devices = NULL;
static int n_motors = 0;


void holger_blmc_init(i2c_bus_t *bus, const uint8_t *addrs, const unsigned int _n_motors)
{
   ASSERT_ONCE();
   n_motors = _n_motors;
   /* allocate memory for device array: */
   devices = malloc(n_motors * sizeof(i2c_dev_t));
   /* initialize devices: */
   FOR_N(i, n_motors)
   {
      i2c_dev_init(&devices[i], bus, addrs[i]);
   }
}


float holger_blmc_characteristic(float force, float voltage, float a, float b)
{
   return powf((force / a * powf(voltage, -1.5f)), 1.0f / b);
}


void holger_blmc_write_read(uint8_t *setpoints, uint8_t *rpm)
{
   ASSERT_NOT_NULL(devices);
   ASSERT_NOT_NULL(rpm);
   FOR_N(i, n_motors)
   {
      i2c_xfer(&devices[i], 1, &setpoints[i], 1, &rpm[i]);
   }
}

