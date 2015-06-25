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
 
 PenguPilot Priority Class Definitions for C/C++

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#ifndef __PP_PRIO_H__
#define __PP_PRIO_H__


#define PP_PRIO_BASE (49)

#define PP_PRIO_1 (PP_PRIO_BASE - 0) /* rotation speed control */
#define PP_PRIO_2 (PP_PRIO_BASE - 1) /* rotation position control */
#define PP_PRIO_3 (PP_PRIO_BASE - 2) /* N/E/U speed control */
#define PP_PRIO_4 (PP_PRIO_BASE - 3) /* N/E/U position control */
#define PP_PRIO_5 (PP_PRIO_BASE - 4) /* missions/manual control */
#define PP_PRIO_6 (PP_PRIO_BASE - 5) /* logging/config/time functionality */


#endif /* __PP_PRIO_H__ */

