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
  
 Modified OMAP3-PWM Multi-ESC Interface

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <util.h>
#include <malloc.h>
#include <stdio.h>

#include "pwm_escs.h"


static size_t _n_escs;
static pwm_esc_t *escs = NULL;


int pwm_escs_init(uint8_t *pwm_ids, size_t n_escs)
{
   ASSERT_ONCE();
   THROW_BEGIN();
   ASSERT_NOT_NULL(pwm_ids);
   ASSERT_TRUE(n_escs > 0);
   escs = malloc(sizeof(pwm_esc_t) * n_escs);
   ASSERT_NOT_NULL(escs);
   FOR_N(i, n_escs)
   {
      char buffer[128];
      snprintf(buffer, sizeof(buffer), "/dev/pwm%d", pwm_ids[i]);
      THROW_ON_ERR(pwm_esc_init(&escs[i], buffer));
   }
   _n_escs = n_escs;
   THROW_END();
}


int pwm_escs_write(float *setpoints)
{
   THROW_BEGIN();
   FOR_N(i, _n_escs)
   {
      THROW_ON_ERR(pwm_esc_write_float(&escs[i], setpoints[i]));
   }
   THROW_END();
}

