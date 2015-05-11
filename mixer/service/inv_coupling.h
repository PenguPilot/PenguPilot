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
  
 Inverse Coupling Matrix Interface

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __INV_COUPLING_H__
#define __INV_COUPLING_H__



#include <motors.h>
#include "coupling.h"


/*
 * "init" array layout:
 *
 * m_1: g_0, p_0, r_0, y_0
 * [...]
 * m_n: g_n, p_n, r_n, y_n
 *
 * g = gas, p = pitch, r = roll, y = yaw, n = #motors
 */
void inv_coupling_init(const size_t n_motors, const float mixer[FORCES_AND_MOMENTS][MAX_MOTORS]);


/*
 * "out" array layout:
 *
 * v_1, ... ,v_n
 * v_x = motor setpoint [0..1]
 * n = #motors
 */
void inv_coupling_calc(float *out, const float *in);


#endif /* __INV_COUPLING__ */

