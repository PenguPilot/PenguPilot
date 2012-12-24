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



#ifndef PID_H
#define PID_H


#include <threadsafe_types.h>


typedef struct
{
   tsfloat_t *p;
   tsfloat_t *i;
   tsfloat_t *d;
   tsfloat_t *max_sum_error;
   tsfloat_t prev_error;
   tsfloat_t sum_error;
}
pid_controller_t;


void pid_init(pid_controller_t *controller, tsfloat_t *p, tsfloat_t *i, tsfloat_t *d, tsfloat_t *max_sum_error);

float pid_control(pid_controller_t *controller, float error, float speed, float dt);

void pid_reset(pid_controller_t *controller);


#endif /* PID_H */

