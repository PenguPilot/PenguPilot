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
  
 Altitude Controller Interface
 
 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __Z_CTRL_H__
#define __Z_CTRL_H__


typedef struct
{
   float val;
   int mode_is_ground;
}
z_setpoint_t;



void z_ctrl_init(float neutral_gas);

void z_ctrl_set_setpoint(z_setpoint_t z_setpoint);

float z_ctrl_get_setpoint(void);

int z_ctrl_mode_is_ground(void);
 
float z_ctrl_step(float *z_err, float ground_z_pos, float z_pos, float speed, float dt);

void z_ctrl_reset(void);


#endif /* __ALT_CTRL_H__ */

