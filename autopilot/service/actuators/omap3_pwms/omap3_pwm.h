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
  
 Modified OMAP3-PWM ESC Interface

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau
 Copyright (C) 2014 Jan Roemisch, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __OMAP3_PWM_H__
#define __OMAP3_PWM_H__


#define OMAP3_PWM_RAW_MIN (10000)
#define OMAP3_PWM_RAW_MAX (20000)

#define OMAP3_PWM_FLOAT_MIN (0.0f)
#define OMAP3_PWM_FLOAT_MAX (1.0f)


typedef struct
{
   int file;
}
omap3_pwm_t;


int omap3_pwm_init(omap3_pwm_t *pwm, char *dev);

int omap3_pwm_write_raw(omap3_pwm_t *pwm, int val);

int omap3_pwm_write_float(omap3_pwm_t *pwm, float val);


#endif /* __OMAP3_PWM_H__ */

