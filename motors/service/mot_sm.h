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
  
 Timer-based Motors State Machine Interface

 States:
 ------------
 0 = stopped
 1 = starting
 2 = running
 3 = stopping

 State chart:
 ------------

                 start()
         .-> [0] ---> [1] --.
 timer() |                  | timer()
         `-- [3] <--- [2] <-´
                 stop()


 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __MOT_SM_H__
#define __MOT_SM_H__


#include <stdbool.h>
#include <mot_state.h>


int mot_sm_init(void);

mot_state_t mot_sm_update(float dt, bool start);


#endif /* __MOT_SM_H__ */

