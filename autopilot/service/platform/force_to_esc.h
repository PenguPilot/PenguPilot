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
  
 Force [N] and Voltage [V] to relative ESC Command Interface

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __FORCE_TO_ESC_H__
#define __FORCE_TO_ESC_H__


/*
 * ESC:       Mikrokopter BL-CTRL 1.2
 * Motor:     Robby Roxxy 2827-35
 * Propeller: 10x4.5 inch
 */
float force_to_esc_setup1(float force, float volt);

/*
 * ESC:       HobbyKing 20A UBEC SimonK
 * Motor:     Robby Roxxy 2827-35
 * Propeller: 10x4.5 inch
 */
float force_to_esc_setup2(float force, float volt);

/*
 * ESC:       Flyduino HEXFET20A SimonK
 * Motor:     Suppo A2212/13
 * Propeller: 10x4.5 inch
 */
float force_to_esc_setup3(float force, float volt);


#endif /* __FORCE_TO_ESC_H__ */

