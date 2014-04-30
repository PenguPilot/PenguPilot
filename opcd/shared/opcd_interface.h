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
  
 OPCD C binding

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __OPCD_INTERFACE_H__
#define __OPCD_INTERFACE_H__


typedef struct
{
   char *id;
   void *data;
}
opcd_param_t;


#define OPCD_PARAMS_END {NULL, NULL}

/*
 * initialize OPCD params
 * prefix is added before all keys defined in the opcd_param_t's
 */
void opcd_params_init(char *prefix, int enable_events);


/*
 * sets a float param to identfied by id to val
 */
void opcd_float_param_set(char *id, float val);


/*
 * 1) retrieves the configuration value from OPCD and store this value
 * 2) registers the parameter for online updates
 */
void opcd_params_apply(char *prefix, opcd_param_t *params);


/*
 * write single parameter into data ptr (if it's a string, memory is allocated)
 */
void opcd_param_get(char *full_name, void *data);


#endif /* __OPCD_INTERFACE_H__ */

