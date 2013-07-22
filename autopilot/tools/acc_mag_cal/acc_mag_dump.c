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
  
 prints acc/mag measurements to standard output
 for later calibration

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include <util.h>
#include <stdio.h>

#include "../../service/platform/drotek_marg2.h"


int main(void)
{
   i2c_bus_t i2c_3;
   i2c_bus_open(&i2c_3, "/dev/i2c-3");
   drotek_marg2_t marg;
   drotek_marg2_init(&marg, &i2c_3);
   printf("acc_x acc_y acc_z mag_x mag_y mag_z\n");
   while (1)
   {
      msleep(10);
      marg_data_t marg_data;
      drotek_marg2_read(&marg_data, &marg);
      printf("%f %f %f %f %f %f\n",
             marg_data.acc.x, marg_data.acc.y, marg_data.acc.z,
             marg_data.mag.x, marg_data.mag.y, marg_data.mag.z);
   }
}

