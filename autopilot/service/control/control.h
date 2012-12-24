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
  
 File Purpose

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


/*
 * File: ctrl.c
 * Type: set of PID controllers
 * Purpose: runs multiple controllers (x,y,z,yaw) and executes attitude control
 * Design Pattern: Singleton using ctrl_init
 *
 * Responsibilities:
 *   - self-configuration through OPCD
 *   - sub-controller management
 *   - controller state management
 *
 * Author: Tobias Simon, Ilmenau University of Technology
 */


#ifndef __CTRL_H__
#define __CTRL_H__

#include <threadsafe_types.h>
#include <pilot.pb-c.h>

#include "../estimators/pos.h"
#include "../geometry/orientation.h"


typedef struct
{
   float pitch;
   float roll;
   float yaw;
   float gas;
}
ctrl_out_t;



typedef struct
{
   tsfloat_t yaw_error;
   tsfloat_t alt_error;
   tsfloat_t x_error;
   tsfloat_t y_error;
}
controller_errors_t;


void ctrl_init(void);


/*
 * reset controllers (affects i-parts)
 */
void ctrl_reset(void);


/*
 * run the controllers
 */
void ctrl_step(ctrl_out_t *out, float dt, pos_t *pos, euler_t *euler);


/*
 * sets the setpoint for type to val
 */
int ctrl_set_data(CtrlParam param, float data);


#endif /* __CTRL_H__ */

