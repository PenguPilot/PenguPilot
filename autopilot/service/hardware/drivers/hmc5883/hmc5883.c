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
  
 HMC5883 Driver Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <errno.h>
#include <string.h>

#include "hmc5883.h"


/* device address */
#define HMC5883_ADDRESS 0x1E

/* registers */
#define HMC5883_CFG_A   0x00
#define HMC5883_CFG_B   0x01
#define HMC5883_CFG_MR  0x02
#define HMC5883_MAGX_H  0x03
#define HMC5883_MAGX_L  0x04
#define HMC5883_MAGY_H  0x05
#define HMC5883_MAGY_L  0x06
#define HMC5883_MAGZ_H  0x07
#define HMC5883_MAGZ_L  0x08
#define HMC5883_STATUS  0x09
#define HMC5883_ID_A    0x0A
#define HMC5883_ID_B    0x0B
#define HMC5883_ID_C    0x0C

/* A register output data rate: */
#define HMC5883_A_ODR_05 0x00
#define HMC5883_A_ODR_1  0x04
#define HMC5883_A_ODR_2  0x08
#define HMC5883_A_ODR_5  0x0C
#define HMC5883_A_ODR_10 0x10
#define HMC5883_A_ODR_20 0x14
#define HMC5883_A_ODR_50 0x18
/* A register measurement config: */
#define HMC5883_A_NORMAL   0x00
#define HMC5883_A_BIAS_POS 0x01
#define HMC5883_A_BIAS_NEG 0x02

/* B register settings: */
#define HMC5883_B_GAIN_0_7 0x00
#define HMC5883_B_GAIN_1   0x20
#define HMC5883_B_GAIN_1_5 0x40
#define HMC5883_B_GAIN_2   0x60
#define HMC5883_B_GAIN_3_2 0x80
#define HMC5883_B_GAIN_3_8 0xA0
#define HMC5883_B_GAIN_4_5 0xC0
#define HMC5883_B_GAIN_6_5 0xE0

/* MR register: */
#define HMC5883_MODE_CONTINUOUS 0x00
#define HMC5883_MODE_SINGLE     0x01
#define HMC5883_MODE_IDLE       0x02
#define HMC5883_MODE_SLEEP      0x02


/* sensitivity conversion table in LSB/Ga */
static float sens_conv_tab[8] =
{
   1602.0, 1300.0, 970.0, 780.0,
   530.0, 460.0, 390.0, 280.0
};
#define CFG_A_2_SENS(v) (sens_conv_tab[((v) >> 5)])



int hmc5883_init(hmc5883_t *hmc, i2c_bus_t *bus)
{
   THROW_BEGIN();
   i2c_dev_init(&hmc->i2c_dev, bus, HMC5883_ADDRESS);
   uint8_t id[3];
   THROW_ON_ERR(i2c_read_block_reg(&hmc->i2c_dev, HMC5883_ID_A, id, sizeof(id)));
   THROW_IF(strncmp("H43", (char *)id, 3) != 0, -ENODEV);
   THROW_ON_ERR(i2c_write_reg(&hmc->i2c_dev, HMC5883_CFG_A, HMC5883_A_ODR_50));
   THROW_ON_ERR(i2c_write_reg(&hmc->i2c_dev, HMC5883_CFG_B, HMC5883_B_GAIN_1));
   THROW_ON_ERR(i2c_write_reg(&hmc->i2c_dev, HMC5883_CFG_MR, HMC5883_MODE_CONTINUOUS));
   THROW_END();
}


int hmc5883_read_mag(float mag[3], hmc5883_t *hmc)
{
   THROW_BEGIN();
   uint8_t data[6];
   THROW_ON_ERR(i2c_read_block_reg(&hmc->i2c_dev, HMC5883_MAGX_H, data, sizeof(data)));
   mag[0] = (int16_t)((data[0] << 8) | data[1]);
   mag[2] = (int16_t)((data[2] << 8) | data[3]);
   mag[1]= (int16_t)((data[4] << 8) | data[5]);
   THROW_END();
}

