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
  
 Stabilizing PIID Controller Interface

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau
 Copyright (C) 2013 Alexander Barth, Control Engineering Group, TU Ilmenau
 Copyright (C) 2013 Benjamin Jahn, Control Engineering Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __PIID_H__
#define __PIID_H__


#define PIID_ROLL  0
#define PIID_PITCH 1
#define PIID_YAW   2


void piid_init(float Ts);

void piid_run(float u_ctrl[3], float gyro[3], float rc[3], float dt);

void piid_int_enable(int val);

void piid_reset(void);


#endif /* __PIID_H__ */

