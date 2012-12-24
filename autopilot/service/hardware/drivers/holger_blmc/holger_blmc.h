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
 * file: holger_blmc.c
 * purpose: mikrokopter brushless motor controller interface
 * author: Tobias Simon, Ilmenau University of Technology
 */


#ifndef __HOLGER_BLMC_H__
#define __HOLGER_BLMC_H__


#include <stdint.h>

#include "../../bus/i2c/i2c.h"


#define HOLGER_I2C_OFF    0
#define HOLGER_I2C_MIN   20
#define HOLGER_I2C_MAX  255



/*
 * creates a holger BLMC interface
 */
void holger_blmc_init(i2c_bus_t *bus, const uint8_t *addrs, const unsigned int n_motors);


/*
 * writes holger BLMC setpoints and reads rpm
 */
void holger_blmc_write_read(uint8_t *setpoints, uint8_t *rpm);


#endif /* __HOLGER_BLMC_H__ */

