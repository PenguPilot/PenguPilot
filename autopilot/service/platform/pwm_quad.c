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
#include "pwm_quad.h"
#include "../hardware/drivers/pwm_esc/pwm_escs.h"


                                     /* m1  m2  m3  m4 */
static uint8_t motor_addrs[N_MOTORS] = { 10, 9, 11, 8};


static float force_to_esc(float force, float voltage)
{
   return 1.66f * (force + 1.5f) / voltage;
}


int pwm_quad_init(platform_t *plat, float f_c)
{
   ASSERT_ONCE();

   /* set-up actuator characteristics: */
   ac_init(&plat->ac, 0.1f, 0.7f, 12.0f, 17.0f, f_c, N_MOTORS, force_to_esc, 0.0f);
    
   /* set-up motors driver: */
   pwm_escs_init(motor_addrs, N_MOTORS);
   plat->write_motors = pwm_escs_write;
   return 0;
}

