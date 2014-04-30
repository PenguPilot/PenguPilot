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
  
 pwm quad platform implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

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
#include "pwm_quad.h"
#include "../hardware/drivers/pwm_esc/pwm_escs.h"
#include "../hardware/drivers/afroi2c_escs/afroi2c_escs.h"


static float force_to_esc(float force, float volt)
{
   if (force < 0)
      return 0;
   const float a = 1.2526e-07;
   const float b = -3.3937e-03;
   const float c = -1.3746e+00;
   const float d = 1.3284e-04;
   const float e = 2.0807e+01;
   float pwm = ((sqrtf((b + d * volt) * (b + d * volt) - 4.0f * a * (c * volt + e - force)) - b - d * volt) / (2.0f * a));
   return (pwm - 10000.0f) / 10000.0f;
}


int pwm_quad_init(platform_t *plat, float f_c)
{
   ASSERT_ONCE();

   /* set-up actuator characteristics: */
   ac_init(&plat->ac, 0.1f, 0.7f, 12.0f, 17.0f, f_c, N_MOTORS, force_to_esc, 0.0f);
    
   /* set-up motors driver: */
   if (1)
   {
      afroi2c_init((i2c_bus_t*)plat->priv, N_MOTORS);
      plat->write_motors = afroi2c_write;
   }
   else
   {
                                            /* m1  m2  m3  m4 */
      static uint8_t motor_addrs[N_MOTORS] = { 10, 9, 11, 8};
      pwm_escs_init(motor_addrs, N_MOTORS);
      plat->write_motors = pwm_escs_write;
   }
   return 0;
}

