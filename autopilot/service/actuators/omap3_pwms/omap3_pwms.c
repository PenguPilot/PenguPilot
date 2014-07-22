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

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

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

#include "omap3_pwm.h"
#include "omap3_pwms.h"


static size_t _n_pwms;
static omap3_pwm_t *pwms = NULL;


int omap3_pwms_init(uint8_t *pwm_ids, size_t n_pwms)
{
   ASSERT_ONCE();
   THROW_BEGIN();
   ASSERT_NOT_NULL(pwm_ids);
   ASSERT_TRUE(n_pwms > 0);
   pwms = malloc(sizeof(omap3_pwm_t) * n_pwms);
   ASSERT_NOT_NULL(pwms);
   FOR_N(i, n_pwms)
   {
      char buffer[128];
      snprintf(buffer, sizeof(buffer), "/dev/pwm%d", pwm_ids[i]);
      THROW_ON_ERR(omap3_pwm_init(&pwms[i], buffer));
   }
   _n_pwms = n_pwms;
   THROW_END();
}


int omap3_pwms_write(float *setpoints)
{
   THROW_BEGIN();
   FOR_N(i, _n_pwms)
   {
      THROW_ON_ERR(omap3_pwm_write_float(&pwms[i], setpoints[i]));
   }
   THROW_END();
}

