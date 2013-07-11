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
  
 Force to Motor Setpoint Converter Interface

 Copyright (C) 2012 Benjamin Jahn, Ilmenau University of Technology
 Copyright (C) 2012 Alexander Barth, Ilmenau University of Technology
 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __FORCE_TO_SETPOINT_H__
#define __FORCE_TO_SETPOINT_H__


typedef struct
{
   float a;
   float b;
   float sp_min;
   float sp_max;
}
force_to_setpoint_t;


int forces_to_setpoints(force_to_setpoint_t *ftos, float *setpoints, float voltage, float *forces, int n_motors);


#endif /* __FORCE_TO_SETPOINT_H__ */

