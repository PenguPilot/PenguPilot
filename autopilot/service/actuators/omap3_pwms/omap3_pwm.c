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
  
 Modified OMAP3-PWM ESC Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology
 Copyright (C) 2014 Jan Roemisch, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>


#include "omap3_pwm.h"


int omap3_pwm_init(omap3_pwm_t *pwm, char *dev)
{
   int f = open(dev, O_RDWR);
   if (f < 0)
   {
      return f;
   }
   pwm->file = f;
   return 0;
}


int omap3_pwm_write_raw(omap3_pwm_t *pwm, int val)
{
   char buffer[10];
   if (val < OMAP3_PWM_RAW_MIN || val > OMAP3_PWM_RAW_MAX)
   {
      return -EINVAL;  
   }
   int len = snprintf(buffer, sizeof(buffer), "%d", val);
   return write(pwm->file, buffer, len);
}


int omap3_pwm_write_float(omap3_pwm_t *pwm, float val)
{
   long int_val = OMAP3_PWM_RAW_MIN + lroundf(val * (float)(OMAP3_PWM_RAW_MAX - OMAP3_PWM_RAW_MIN));
   return omap3_pwm_write_raw(pwm, int_val);
}

