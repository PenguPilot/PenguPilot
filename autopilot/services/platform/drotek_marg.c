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
   DROTEK MARG platform driver - implementation

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


#include <util.h>


#include "drotek_marg.h"


int drotek_marg_init(drotek_marg_t *marg, i2c_bus_t *bus)
{
   THROW_BEGIN();
   THROW_ON_ERR(itg3200_init(&marg->itg, bus, ITG3200_DLPF_98HZ));
   THROW_ON_ERR(bma180_init(&marg->bma, bus, BMA180_RANGE_4G, BMA180_BW_40HZ));
   THROW_ON_ERR(hmc5883_init(&marg->hmc, bus));
   THROW_END();
}


int drotek_marg_read(marg_data_t *data, drotek_marg_t *marg)
{
   THROW_BEGIN();
   THROW_ON_ERR(itg3200_read_gyro(data->gyro.vec, &marg->itg));
   THROW_ON_ERR(bma180_read_acc(data->acc.vec, &marg->bma));
   THROW_ON_ERR(hmc5883_read_mag(data->mag.vec, &marg->hmc));
   THROW_END();
}


