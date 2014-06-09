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
  
 Timer-based Motors State Tracking

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __MOTORS_STATE_H__
#define __MOTORS_STATE_H__


#include <stdbool.h>


/* initializes motor state */
void motors_state_init(void);


/* indicates if the motors are starting */
bool motors_starting(void);


/* indicates if the motor outputs should be disabled */
bool motors_output_is_disabled(void);


/* indicates if the motors are spinning */
bool motors_spinning(void);


/* indicates if the controller inputs are used  */
bool motors_controllable(void);


/* updates the motor state machine */
void motors_state_update(bool flying, float dt, bool start);


#endif /* __MOTORS_STATE_H__ */

