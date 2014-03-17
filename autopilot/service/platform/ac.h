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
  
 Actuator Characteristic - Interface

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __AC_H__
#define __AC_H__


#include <stddef.h>


/* linear or non-linear function mapping a force and a voltage to a motor setpoint */
typedef float (*acf_t)(const float force, const float voltage);


/* actuator characteristics */
typedef struct
{
   float min; /* lower limit for func output */
   float max; /* upper limit for func output */
   float v_min; /* lower limit for voltage */
   float v_max; /* upper limit for voltage */
   float f_c; /* factor to map rpm^2 to a force in N */
   size_t n_motors; /* number of motors */
   acf_t func; /* see acf_t definition */
   float off_val; /* value overriding the func output, signaling motors off */
}
ac_t;


/* initializes actuator characteristic */
void ac_init(ac_t *ac, float min, float max, float v_min, float v_max,
             float f_c, size_t n_motors, acf_t func, float off_val);


/* compute actuator setpoints from desired rpm_square values
   returns 1, if one or more actuator is saturated and 0 otherwise */
int ac_calc(float *out, const ac_t *ac, const int enabled,
            const float voltage, const float *rpm_sq);


#endif /* __AC_H__ */

