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


/*
   I2C Linux Interface

   Copyright (C) 2012 Tobias Simon

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
*/


#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>

#include "i2c.h"
#include "i2c-dev.h"


int i2c_bus_open(i2c_bus_t *bus, char *path)
{
   THROW_BEGIN();
   THROW_ON_ERR(open(path, O_RDWR));
   bus->dev_addr = 0xFF; /* invalid i2c addess in 7 bit mode */
   bus->handle = THROW_PREV;
   THROW_ON_ERR(pthread_mutex_init(&bus->mutex, NULL));
   THROW_END();
}


int i2c_bus_close(i2c_bus_t *bus)
{
   THROW_PROPAGATE(close(bus->handle));
}


void i2c_dev_init(i2c_dev_t *dev, i2c_bus_t *bus, uint8_t addr)
{
   dev->bus = bus;
   dev->addr = addr;
}


static void i2c_dev_lock_bus(i2c_dev_t *dev)
{
   pthread_mutex_lock(&dev->bus->mutex);
}


static void i2c_dev_unlock_bus(i2c_dev_t *dev)
{
   pthread_mutex_unlock(&dev->bus->mutex);
}


static int set_slave_address_if_needed(i2c_dev_t *dev)
{
   THROW_BEGIN();
   if (dev->bus->dev_addr != dev->addr)
   {
      THROW_ON_ERR(ioctl(dev->bus->handle, I2C_SLAVE, dev->addr));
      dev->bus->dev_addr = dev->addr;
   }
   THROW_END();
}


int i2c_write(i2c_dev_t *dev, uint8_t val)
{
   THROW_BEGIN();
   i2c_dev_lock_bus(dev);
   THROW_ON_ERR(set_slave_address_if_needed(dev));
   THROW_ON_ERR(i2c_smbus_write_byte(dev->bus->handle, val));
   THROW_END_EXEC(i2c_dev_unlock_bus(dev));
}


int i2c_write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
   THROW_BEGIN();
   i2c_dev_lock_bus(dev);
   THROW_ON_ERR(set_slave_address_if_needed(dev));
   THROW_ON_ERR(i2c_smbus_write_byte_data(dev->bus->handle, reg, val));
   THROW_END_EXEC(i2c_dev_unlock_bus(dev));
}


int i2c_rdwr(i2c_dev_t *dev, uint8_t len_wr, uint8_t *wr_data, uint8_t len_rd, uint8_t *rd_data)
{
    THROW_BEGIN();
    struct i2c_rdwr_ioctl_data  msgset;
    struct i2c_msg msgs[2];

    msgs[0].addr    =   dev->addr;
    msgs[0].len     =   len_wr;
    msgs[0].flags   =   I2C_SMBUS_WRITE;
    msgs[0].buf     =   (char *)wr_data;

    msgs[1].addr    =   dev->addr;
    msgs[1].flags   =   I2C_SMBUS_READ;
    msgs[1].len     =   len_rd;
    msgs[1].buf     =   (char *)rd_data;

    msgset.nmsgs    =   2;
    msgset.msgs     =     msgs;

    i2c_dev_lock_bus(dev);
    THROW_ON_ERR(ioctl(dev->bus->handle, I2C_RDWR, &msgset));
    THROW_END_EXEC(i2c_dev_unlock_bus(dev));
}


int i2c_read(i2c_dev_t *dev)
{
   THROW_BEGIN();
   i2c_dev_lock_bus(dev);
   THROW_ON_ERR(set_slave_address_if_needed(dev));
   THROW_ON_ERR(i2c_smbus_read_byte(dev->bus->handle));
   THROW_END_EXEC(i2c_dev_unlock_bus(dev)); /* this also returns the data byte */
}


int i2c_read_reg(i2c_dev_t *dev, uint8_t reg)
{
   THROW_BEGIN();
   i2c_dev_lock_bus(dev);
   THROW_ON_ERR(set_slave_address_if_needed(dev));
   THROW_ON_ERR(i2c_smbus_read_byte_data(dev->bus->handle, reg));
   THROW_END_EXEC(i2c_dev_unlock_bus(dev)); /* this also returns the data byte */
}


int i2c_read_block_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *buf, size_t len)
{
   THROW_BEGIN();
   i2c_dev_lock_bus(dev);
   THROW_ON_ERR(set_slave_address_if_needed(dev));
   THROW_ON_ERR(i2c_smbus_read_i2c_block_data(dev->bus->handle, reg, len, buf) == (int)len ? 0 : -EIO)
   THROW_END_EXEC(i2c_dev_unlock_bus(dev));
}




