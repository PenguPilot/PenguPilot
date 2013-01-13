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

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology
 Copyright (C) 2012 Jan Roemisch, Ilmenau University of Technology

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


#include "pwm_esc.h"


int pwm_esc_init(pwm_esc_t *esc, char *dev)
{
   int f = open(dev, O_RDWR);
   if (f < 0)
   {
      return f;
   }
   esc->file = f;
   return 0;
}


int pwm_esc_write_raw(pwm_esc_t *esc, int val)
{
   char buffer[10];
   if (val < PWM_ESC_RAW_MIN || val > PWM_ESC_RAW_MAX)
   {
      return -EINVAL;  
   }
   int len = snprintf(buffer, sizeof(buffer), "%d", val);
   return write(esc->file, buffer, len);
}


int pwm_esc_write_float(pwm_esc_t *esc, float val)
{
   long int_val = PWM_ESC_RAW_MIN + roundf(val * (float)(PWM_ESC_RAW_MAX - PWM_ESC_RAW_MIN));
   return pwm_esc_write_raw(esc, int_val);
}

