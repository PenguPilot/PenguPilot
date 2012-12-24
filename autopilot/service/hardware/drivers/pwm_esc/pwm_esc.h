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

 Copyright (C) 2012 Tobias Simon,
                    Jan Roemisch, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __PWM_ESC_H__
#define __PWM_ESC_H__


#define PWM_ESC_RAW_MIN (10000)
#define PWM_ESC_RAW_MAX (10000)

#define PWM_ESC_FLOAT_MIN (0.0f)
#define PWM_ESC_FLOAT_MAX (0.0f)


typedef struct
{
   int file;
}
pwm_esc_t;


int pwm_esc_init(pwm_esc_t *esc, char *dev);

int pwm_esc_write_raw(pwm_esc_t *esc, int val);

int pwm_esc_write_float(pwm_esc_t *esc, float val);


#endif /* __PWM_ESC_H__ */

