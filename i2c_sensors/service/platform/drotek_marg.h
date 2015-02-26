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
  
 DROTEK MARG (ITG) Driver Interface

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __DROTEK_MARG_H__
#define __DROTEK_MARG_H__


#include <util.h>

#include "../sensors/util/marg_data.h"
#include "../sensors/itg3200/itg3200.h"
#include "../sensors/bma180/bma180.h"
#include "../sensors/hmc5883/hmc5883.h"


typedef struct
{
   itg3200_t itg;
   bma180_t bma;
   hmc5883_t hmc;
}
drotek_marg_t;


int drotek_marg_init(drotek_marg_t *marg, i2c_bus_t *bus);

int drotek_marg_read(marg_data_t *data, drotek_marg_t *marg);


#endif /* __DROTEK_MARG_H__ */

