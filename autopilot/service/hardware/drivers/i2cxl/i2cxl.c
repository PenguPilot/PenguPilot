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
  
 MaxSonar I2CXL Driver Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "i2cxl.h"

#include <util.h>


#define I2CXL_ADDRESS       0x70
#define I2CXL_RANGE_COMMAND 0x51
#define I2CXL_READ          0x02
#define I2CXL_MIN_RANGE     0.2f
#define I2CXL_MAX_RANGE     7.0f
#define I2CXL_M_SCALE       1.0e-2f


int i2cxl_init(i2cxl_t *i2cxl, i2c_bus_t *bus)
{
   /* copy values */
   i2c_dev_init(&i2cxl->i2c_dev, bus, I2CXL_ADDRESS);
   return 0;
}


int i2cxl_read(i2cxl_t *i2cxl, float *dist)
{
   THROW_BEGIN();
   /* start measurement: */
   THROW_ON_ERR(i2c_write(&i2cxl->i2c_dev, I2CXL_RANGE_COMMAND));

   msleep(70);
   
   /* read back the result: */
   uint8_t raw[2];
   THROW_ON_ERR(i2c_read_block_reg(&i2cxl->i2c_dev, I2CXL_READ, raw, sizeof(raw)));
   *dist = limit(I2CXL_M_SCALE * (float)((raw[0] << 8) | raw[1]), I2CXL_MIN_RANGE, I2CXL_MAX_RANGE);
   THROW_END();
}

