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
  
 ITG3200 Driver Implementation

 Copyright (C) 2012 Jan Römisch, Ilmenau University of Technology
 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#include "itg3200.h"


#define ITG3200_ADDRESS 0x69

#define ITG3200_GYRO_INIT_COUNT 1000
#define ITG3200_WHO_AM_I       0x00
#define ITG3200_SMPLRT_DIV     0x15

#define ITG3200_DLPF_FS             0x16
#define ITG3200_DLPF_FS_DLPF_CFG(x) ((x) & 0x7)
#define ITG3200_DLPF_FS_FS_SEL(x)   (((x) & 0x3) << 3)

#define ITG3200_INT_CFG                      0x17
#define ITG3200_INT_CFG_RAW_RDY_EN           (1 << 0)
#define ITG3200_INT_CFG_ITG_RDY_EN           (1 << 2)
#define ITG3200_INT_CFG_INT_ANYRD_RDY_2CLEAR (1 << 4)
#define ITG3200_INT_CFG_LATCH_INT_EN         (1 << 5)
#define ITG3200_INT_CFG_OPEN                 (1 << 6)
#define ITG3200_INT_CFG_ACTL                 (1 << 7)

#define ITG3200_INT_STATUS              0x1A
#define ITG3200_INT_STATUS_RAW_DATA_RDY (1 << 0)
#define ITG3200_INT_STATUS_ITG_RDY      (1 << 2)

#define ITG3200_TEMP_OUT_H 0x1B
#define ITG3200_TEMP_OUT_L 0x1C

#define ITG3200_GYRO_XOUT_H 0x1D
#define ITG3200_GYRO_XOUT_L 0x1E
#define ITG3200_GYRO_YOUT_H 0x1F
#define ITG3200_GYRO_YOUT_L 0x20
#define ITG3200_GYRO_ZOUT_H 0x21
#define ITG3200_GYRO_ZOUT_L 0x22

#define ITG3200_PWR_MGM            0x3E
#define ITG3200_PWR_MGM_H_RESET    (1 << 7)
#define ITG3200_PWR_MGM_SLEEP      (1 << 6)
#define ITG3200_PWR_MGM_STBY_XG    (1 << 5)
#define ITG3200_PWR_MGM_STBY_YG    (1 << 4)
#define ITG3200_PWR_MGM_STBY_ZG    (1 << 3)
#define ITG3200_PWR_MGM_CLK_SEL(x) ((x) & 0x3)



static int read_gyro_raw(itg3200_t *itg, int16_t *data)
{
   THROW_BEGIN();

   /* read gyro registers */
   uint8_t raw[6];
   THROW_ON_ERR(i2c_read_block_reg(&itg->i2c_dev, ITG3200_GYRO_XOUT_H, raw, sizeof(raw)));

   int i;
   for(i = 0; i < 3; i++)
   {
      data[i] = (int16_t)((raw[(i << 1)] << 8) | raw[(i << 1) + 1]);
   }

   THROW_END();
}


int itg3200_init(itg3200_t *itg, i2c_bus_t *bus, itg3200_dlpf_t filter)
{
   THROW_BEGIN();

   /* copy values */
   i2c_dev_init(&itg->i2c_dev, bus, ITG3200_ADDRESS);

   /* verify chip identificatio:n */
   THROW_ON_ERR(i2c_read_reg(&itg->i2c_dev, ITG3200_WHO_AM_I));
   THROW_IF((uint8_t)THROW_PREV != itg->i2c_dev.addr, -ENODEV);

   /* reset device: */
   THROW_ON_ERR(i2c_write_reg(&itg->i2c_dev, ITG3200_PWR_MGM, ITG3200_PWR_MGM_H_RESET));

   /* wait 70 ms for gyro startup: */
   msleep(70);

   /* set z-gyro as clock source: */
   THROW_ON_ERR(i2c_write_reg(&itg->i2c_dev, ITG3200_PWR_MGM, ITG3200_PWR_MGM_CLK_SEL(0x3)));

   /* set full scale mode and low-pass filter: */
   THROW_ON_ERR(i2c_write_reg(&itg->i2c_dev, ITG3200_DLPF_FS, ITG3200_DLPF_FS_FS_SEL(0x3) | ITG3200_DLPF_FS_DLPF_CFG(filter)));

   THROW_END();
}


int itg3200_read_gyro(float gyro[3], itg3200_t *itg)
{
   THROW_BEGIN();
   
   /* read raw gyro data: */
   int16_t raw[3];
   THROW_ON_ERR(read_gyro_raw(itg, raw));

   /* construct, scale and bias-correct values: */
   FOR_N(i, 3)
   {
      gyro[i] = ((float)raw[i] / 14.375) * M_PI / 180.0;
   }

   THROW_END();
}


int itg3200_read_temperature(float *temperature, itg3200_t *itg)
{
   THROW_BEGIN();

   /* read temperature registers: */
   uint8_t raw[2];
   THROW_ON_ERR(i2c_read_block_reg(&itg->i2c_dev, ITG3200_TEMP_OUT_H, raw, sizeof(raw)));

   /* construct and scale value: */
   *temperature = (3500.0 + (float)((int16_t)(raw[0] << 8 | raw[1]) + 13200) / 2.80) / 100.0;
   
   THROW_END();
}

