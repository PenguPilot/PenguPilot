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
  
 AK8975C Driver Implementation

 Copyright (C) 2014 Jan Roemisch, Ilmenau University of Technology
 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include <util.h>

#include "ak8975c.h"

#define AK8975C_ADDRESS    0x0C

#define AK8975C_WIA        0x00
#define AK8975C_INFO       0x01

#define AK8975C_ST1        0x02
#define AK8975C_ST1_DRDY   (1 << 0)

#define AK8975C_HXL        0x03
#define AK8975C_HXH        0x04
#define AK8975C_HYL        0x05
#define AK8975C_HYH        0x06
#define AK8975C_HZL        0x07
#define AK8975C_HZH        0x08

#define AK8975C_ST2        0x09
#define AK8975C_ST2_HOFL   (1 << 3)
#define AK8975C_ST2_DERR   (1 << 2)

#define AK8975C_CNTL       0x0A

#define AK8975C_RSV        0x0B

#define AK8975C_ASTC       0x0C
#define AK8975C_ASTC_SELF  (1 << 6)

#define AK8975C_TS1        0x0D
#define AK8975C_TS2        0x0E

#define AK8975C_I2CDIS     0x0F
#define AK8975C_I2CDIS_I2CDIS (1 << 0)

#define AK8975C_ASAX       0x10
#define AK8975C_ASAY       0x11
#define AK8975C_ASAZ       0x12


static int read_raw(ak8975c_dev_t *dev)
{
   THROW_BEGIN();
   uint8_t data[6];
   
   /* check for data ready */
   THROW_ON_ERR(i2c_read_reg(&dev->i2c_dev, AK8975C_ST1));
   if (!(THROW_PREV & AK8975C_ST1_DRDY))
      THROW(-EAGAIN);
   
   /* read out values */
   THROW_ON_ERR(i2c_read_block_reg(&dev->i2c_dev, AK8975C_HXL, data, sizeof(data)));

   dev->raw.x = (int16_t)((data[1] << 8) | data[0]);
   dev->raw.y = (int16_t)((data[3] << 8) | data[2]);
   dev->raw.z = (int16_t)((data[5] << 8) | data[4]);

   THROW_END();
}


static int self_test(ak8975c_dev_t *dev)
{
   THROW_BEGIN();
   
   /* Power down */
   THROW_ON_ERR(i2c_write_reg(&dev->i2c_dev, AK8975C_CNTL, 0x0));
   
   /* Generate magnetic field for self test */
   THROW_ON_ERR(i2c_write_reg(&dev->i2c_dev, AK8975C_ASTC, AK8975C_ASTC_SELF));

   /* Set self test mode */
   THROW_ON_ERR(i2c_write_reg(&dev->i2c_dev, AK8975C_CNTL, 0x8));

   /* wait for data ready */
   while (read_raw(dev) < 0) {}

   /* disable magnetic field generation */
   THROW_ON_ERR(i2c_write_reg(&dev->i2c_dev, AK8975C_ASTC, 0));

   /* check values */
   if (dev->raw.x < -100. || dev->raw.x > 100.)
   {
      THROW(-ENODEV);
   }
   if (dev->raw.y < -100. || dev->raw.y > 100.)
   {
      THROW(-ENODEV);
   }
   if (dev->raw.z < -1000. || dev->raw.z > -300.)
   {
      THROW(-ENODEV);
   }

   THROW_END();
}


int ak8975c_init(ak8975c_dev_t *dev, i2c_bus_t *bus)
{
   THROW_BEGIN();
   vec3_init(&dev->raw);
   i2c_dev_init(&dev->i2c_dev, bus, AK8975C_ADDRESS);

   THROW_ON_ERR(i2c_read_reg(&dev->i2c_dev, AK8975C_WIA));
   THROW_IF(THROW_PREV != 0x48, -ENODEV);

   THROW_ON_ERR(self_test(dev));
   THROW_END();
}


int ak8975c_read(ak8975c_dev_t *dev)
{
   THROW_BEGIN();

   /* start measurement */
   THROW_ON_ERR(i2c_write_reg(&dev->i2c_dev, AK8975C_CNTL, 0x1));
   msleep(10);

   /* read values */
   THROW_ON_ERR(read_raw(dev));

   /* check validity */
   THROW_ON_ERR(i2c_read_reg(&dev->i2c_dev, AK8975C_ST2));
   THROW_IF(THROW_PREV & (AK8975C_ST2_HOFL | AK8975C_ST2_DERR), -EIO);

   THROW_END();
}

