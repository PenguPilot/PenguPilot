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
  
 HMC5883 Driver Interface

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __HMC5883_H__
#define __HMC5883_H__


#include <stdint.h>

#include <util.h>

#include <i2c/i2c.h>
#include <math/vec3.h>


typedef struct
{
   /* i2c device: */
   i2c_dev_t i2c_dev;
   /* privated data: */
   uint8_t gain;
   vec3_t prev;
}
hmc5883_t;


int hmc5883_init(hmc5883_t *dev, i2c_bus_t *bus);

int hmc5883_read_mag(vec3_t *mag, const hmc5883_t *dev);


#endif /* __HMC5883_H__ */

