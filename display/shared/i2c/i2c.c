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
  
 Threadsafe Linux I2C Implementation

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>

#include "i2c.h"
#include "i2c-dev.h"


int i2c_bus_open(i2c_bus_t *bus, char *path)
{
   assert(bus);
   assert(path);
   int err = open(path, O_RDWR);
   if (err < 0)
      goto out;
   *bus = err;
   err = 0;
out:
   return err;
}


int i2c_bus_close(i2c_bus_t *bus)
{
   assert(bus);
   return close(*bus);
}


void i2c_dev_init(i2c_dev_t *dev, i2c_bus_t *bus, uint8_t addr)
{
   assert(dev);
   assert(bus);
   dev->bus = bus;
   dev->addr = addr;
}


int i2c_xfer(i2c_dev_t *dev, uint8_t len_wr, uint8_t *wr_data, uint8_t len_rd, uint8_t *rd_data)
{
   assert(dev);
   struct i2c_rdwr_ioctl_data msgset;
   struct i2c_msg msgs[2];

   int i = 0;
   if (wr_data)
   {
      msgs[i].addr = dev->addr;
      msgs[i].len = len_wr;
      msgs[i].flags = I2C_SMBUS_WRITE;
      msgs[i].buf = (char *)wr_data;
      i += 1;
   }

   if (rd_data)
   {
      msgs[i].addr = dev->addr;
      msgs[i].flags = I2C_SMBUS_READ;
      msgs[i].len  = len_rd;
      msgs[i].buf = (char *)rd_data;
      msgset.nmsgs = i + 1;
      i += 1;
   }
   msgset.nmsgs = i;
   msgset.msgs = msgs;

   return ioctl(*dev->bus, I2C_RDWR, &msgset) == i ? 0 : -EIO;
}


int i2c_write(i2c_dev_t *dev, uint8_t val)
{
   return i2c_xfer(dev, 1, &val, 0, NULL);
}


int i2c_write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
   uint8_t buf[2] = {reg, val};
   return i2c_xfer(dev, 2, buf, 0, NULL);
}


int i2c_read(i2c_dev_t *dev)
{
   uint8_t buf;
   return i2c_xfer(dev, 0, NULL, 1, &buf);
}


int i2c_read_reg(i2c_dev_t *dev, uint8_t reg)
{
   uint8_t buf;
   int err = i2c_xfer(dev, 1, &reg, 1, &buf);
   if (err < 0)
      return err;
   return buf;
}


int i2c_read_block_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *buf, size_t len)
{
   return i2c_xfer(dev, 1, &reg, len, buf);
}

