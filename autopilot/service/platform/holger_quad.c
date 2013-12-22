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
  
 holger quad platform implementation

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <math.h>
#include <util.h>

#include "quad.h"
#include "holger_quad.h"
#include "../hardware/drivers/holger_blmc/holger_blmc.h"


                                     /* m0    m1    m2    m3 */
static uint8_t motor_addrs[N_MOTORS] = {0x29, 0x2a, 0x2b, 0x2c};


static int write_motors(float *setpoints)
{
   uint8_t rpm[N_MOTORS];
   uint8_t u8setp[N_MOTORS];
   FOR_N(i, N_MOTORS)
   {
      u8setp[i] = round(setpoints[i]);
   }
   holger_blmc_write_read(u8setp, rpm);
   return 0; // TODO implement
}


static float force_to_blmc(float force, float voltage)
{
   float a = 609.6137f;
   float b = 1.3154f;
   return powf((force / a * powf(voltage, -1.5f)), 1.0f / b);
}


int holger_quad_init(platform_t *plat, float f_c)
{
   ASSERT_ONCE();

   /* set-up actuator characteristics: */
   ac_init(&plat->ac, HOLGER_I2C_MIN, HOLGER_I2C_MAX, 12.0, 17.0, f_c, N_MOTORS, force_to_blmc, 0);
    
   /* set-up motors driver: */
   holger_blmc_init((i2c_bus_t *)plat->priv, motor_addrs, N_MOTORS);
   plat->write_motors = write_motors;
   return 0;
}

