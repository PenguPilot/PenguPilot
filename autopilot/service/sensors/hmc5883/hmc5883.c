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

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

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
#define HMC5883_MAGZ_H  0x05
#define HMC5883_MAGZ_L  0x06
#define HMC5883_MAGY_H  0x07
#define HMC5883_MAGY_L  0x08
#define HMC5883_STATUS  0x09
#define HMC5883_ID_A    0x0A
#define HMC5883_ID_B    0x0B
#define HMC5883_ID_C    0x0C

/* status register values: */
#define HMC5883_STATUS_LOCK 0x02
#define HMC5883_STATUS_RDY 0x01


/* A register measurements averaging: */
#define HMC5883_A_MA_1 (0x00 << 5)
#define HMC5883_A_MA_2 (0x01 << 5)
#define HMC5883_A_MA_4 (0x02 << 5)
#define HMC5883_A_MA_8 (0x03 << 5)
/* A register output data rate: */
#define HMC5883_A_DO_0_7 (0x0 << 2)
#define HMC5883_A_DO_1_5 (0x1 << 2)
#define HMC5883_A_DO_3   (0x2 << 2)
#define HMC5883_A_DO_7_5 (0x3 << 2)
#define HMC5883_A_DO_15  (0x4 << 2)
#define HMC5883_A_DO_30  (0x5 << 2)
#define HMC5883_A_DO_75  (0x6 << 2)
/* A register measurement config: */
#define HMC5883_A_MS_NORMAL   (0x00 << 0)
#define HMC5883_A_MS_BIAS_POS (0x01 << 0)
#define HMC5883_A_MS_BIAS_NEG (0x02 << 0)

/* B register settings: */
#define HMC5883_B_GN_0_8 (0x0 << 5)
#define HMC5883_B_GN_1_3 (0x1 << 5)
#define HMC5883_B_GN_1_9 (0x2 << 5)
#define HMC5883_B_GN_2_5 (0x3 << 5)
#define HMC5883_B_GN_4_0 (0x4 << 5)
#define HMC5883_B_GN_4_7 (0x5 << 5)
#define HMC5883_B_GN_5_6 (0x6 << 5)
#define HMC5883_B_GN_8_1 (0x7 << 5)

/* MR register: */
#define HMC5883_MODE_CONTINUOUS 0x00
#define HMC5883_MODE_SINGLE     0x01
#define HMC5883_MODE_IDLE       0x02
#define HMC5883_MODE_SLEEP      0x02


/* sensitivity conversion table in LSB/Ga */
static float sens_conv_tab[8] =
{
   0.73e-3,
   0.92e-3,
   1.22e-3,
   1.52e-3,
   2.27e-3,
   2.56e-3,
   3.03e-3,
   4.35e-3
};


int hmc5883_init(hmc5883_t *hmc, i2c_bus_t *bus)
{
   vec3_init(&hmc->prev);
   hmc->gain = HMC5883_B_GN_4_0;
   THROW_BEGIN();
   i2c_dev_init(&hmc->i2c_dev, bus, HMC5883_ADDRESS);
   uint8_t id[3];
   THROW_ON_ERR(i2c_read_block_reg(&hmc->i2c_dev, HMC5883_ID_A, id, sizeof(id)));
   THROW_IF(strncmp("H43", (char *)id, 3) != 0, -ENODEV);
   THROW_ON_ERR(i2c_write_reg(&hmc->i2c_dev, HMC5883_CFG_A, HMC5883_A_MA_8 | HMC5883_A_DO_75 | HMC5883_A_MS_NORMAL));
   THROW_ON_ERR(i2c_write_reg(&hmc->i2c_dev, HMC5883_CFG_B, hmc->gain));
   THROW_ON_ERR(i2c_write_reg(&hmc->i2c_dev, HMC5883_CFG_MR, HMC5883_MODE_CONTINUOUS));
   THROW_END();
}


int hmc5883_read_mag(vec3_t *mag, const hmc5883_t *hmc)
{
   THROW_BEGIN();
   THROW_ON_ERR(i2c_read_reg(&hmc->i2c_dev, HMC5883_STATUS));
   if (!(THROW_PREV & HMC5883_STATUS_RDY))
   {
      THROW_PREV = 0;
      goto out; /* no new data; use previous measurement at "out" label */
   }

   /* update device data if we have new data: */
   uint8_t data[6];
   THROW_ON_ERR(i2c_read_block_reg(&hmc->i2c_dev, HMC5883_MAGX_H, data, sizeof(data)));
   hmc->prev.ve[0] = (int16_t)((data[0] << 8) | data[1]);
   hmc->prev.ve[2] = (int16_t)((data[2] << 8) | data[3]);
   hmc->prev.ve[1] = (int16_t)((data[4] << 8) | data[5]);
   FOR_N(i, 3)
      hmc->prev.ve[i] *= sens_conv_tab[hmc->gain >> 5];
out:
   vec_copy(mag, &hmc->prev);
   THROW_END();
}

