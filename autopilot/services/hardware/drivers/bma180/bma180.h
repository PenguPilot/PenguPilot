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
  
 BMA180 Accelerometer Driver

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


#ifndef __BMA180_H__
#define __BMA180_H__


#include <stdint.h>

#include <util.h>

#include "../../bus/i2c/i2c.h"
#include "../../../geometry/orientation.h"


/* accelerometer range: */
typedef enum 
{
   BMA180_RANGE_1G,
   BMA180_RANGE_1_5G,
   BMA180_RANGE_2G,
   BMA180_RANGE_3G,
   BMA180_RANGE_4G,
   BMA180_RANGE_8G,
   BMA180_RANGE_16G
}
bma180_range_t;


/* filter configuration: */
typedef enum
{
   /* low-pass: */
   BMA180_BW_10HZ,
   BMA180_BW_20HZ,
   BMA180_BW_40HZ,
   BMA180_BW_75HZ,
   BMA180_BW_150HZ,
   BMA180_BW_300HZ,
   BMA180_BW_600HZ,
   BMA180_BW_1200HZ,

   /* highpass: 1Hz */
   BMA180_BW_HP,

   /* band-pass: 0.2Hz ... 300Hz */
   BMA180_BW_BP
}
bma180_bw_t;


typedef struct
{
   /* i2c device: */
   i2c_dev_t i2c_dev;

   /* static information */
   uint8_t chip_id;
   uint8_t version;
	
   bma180_range_t range;
	
   /* offset: */
   union
   {
      struct
      {
         int8_t t;
         int16_t x;
         int16_t y;
         int16_t z;
      };
      int16_t data[4];
   }
   offset;

   /* gain: */
   union 
   {
      struct
      {
         int8_t t;
         int8_t x;
         int8_t y;
         int8_t z;
      };
      int8_t data[4];
   } 
   gain;
}
bma180_t;


int bma180_init(bma180_t *bma, i2c_bus_t *bus, bma180_range_t range, bma180_bw_t bandwidth);

int bma180_read_acc(float acc[3], bma180_t *bma);

int bma180_read_temperature(float *temperature, bma180_t *bma);


#endif /* __BMA180_H__ */

