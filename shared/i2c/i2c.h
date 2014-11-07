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
  
 Threadsafe Linux I2C Interface

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __I2C_H__
#define __I2C_H__


#include <stdint.h>
#include <stddef.h>


/* bus type definition: */
typedef int i2c_bus_t;


/* device type definition: */
typedef struct
{
   i2c_bus_t *bus;
   uint8_t addr;
}
i2c_dev_t;


int i2c_bus_open(i2c_bus_t *bus, const char *path);
int i2c_bus_close(i2c_bus_t *bus);
void i2c_dev_init(i2c_dev_t *dev, i2c_bus_t *bus, const uint8_t addr);

int i2c_xfer(const i2c_dev_t *dev, const uint8_t len_wr, const uint8_t *wr_data, const uint8_t len_rd, uint8_t *rd_data);
int i2c_write(const i2c_dev_t *dev, const uint8_t val);
int i2c_write_reg(const i2c_dev_t *dev, const uint8_t reg, const uint8_t val);
int i2c_read(const i2c_dev_t *dev);
int i2c_read_reg(const i2c_dev_t *dev, const uint8_t reg);
int i2c_read_block_reg(const i2c_dev_t *dev, const uint8_t reg, uint8_t *buf, const size_t len);


#endif /* __I2C_H__ */

