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
  
 Flight Logic Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "flight_logic.h"
#include "man_logic.h"
#include "auto_logic.h"
#include "../main_loop/control_mode.h"


static enum
{
   MODE_MANUAL,
   MODE_SAFE_AUTO,
   MODE_FULL_AUTO
}
flight_mode = MODE_FULL_AUTO;


void flight_logic_init(void)
{
   switch (flight_mode)
   {
      case MODE_MANUAL:
         man_logic_init();
         break;

      case MODE_SAFE_AUTO:
         auto_logic_init();
         break;

      case MODE_FULL_AUTO:
         auto_logic_init();
         break;
   }
}


bool flight_logic_run(bool *hard_off,
                      uint16_t sensor_status,
                      bool flying,
                      float channels[MAX_CHANNELS],
                      float yaw,
                      vec2_t *ne_gps_pos,
                      float u_baro_pos,
                      float u_ultra_pos,
                      float f_max,
                      float mass,
                      float dt)

{
   bool motors_enabled = false;
   switch (flight_mode)
   {
      case MODE_MANUAL:
         motors_enabled = man_logic_run(hard_off, sensor_status, flying, channels, yaw, ne_gps_pos, u_baro_pos, u_ultra_pos, f_max, mass, dt);
         break;

      case MODE_SAFE_AUTO:
         motors_enabled = auto_logic_run(hard_off, false, sensor_status, flying, channels, yaw, ne_gps_pos, u_baro_pos, u_ultra_pos);
         break;

      case MODE_FULL_AUTO:
         motors_enabled = auto_logic_run(hard_off, true, sensor_status, flying, channels, yaw, ne_gps_pos, u_baro_pos, u_ultra_pos);
         break;

   }
   return motors_enabled;
}

