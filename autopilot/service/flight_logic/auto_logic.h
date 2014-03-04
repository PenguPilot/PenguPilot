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
  
 Auto Flight Logic Interface

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __AUTO_LOGIC_H__
#define __AUTO_LOGIC_H__


#include <stdbool.h>
#include <stdint.h>
#include "../hardware/util/rc_channels.h"
#include "../util/math/vec2.h"

/* initialize autoflight logic */
void auto_logic_init(void);


/* run auto flight logic */
void auto_logic_run(bool is_full_auto,
                    uint16_t sensor_status,
                    bool flying,
                    float channels[MAX_CHANNELS],
                    float yaw, vec2_t *ne_gps_pos,
                    float u_baro_pos,
                    float u_ultra_pos);


#endif /* __AUTO_LOGIC_H__ */

