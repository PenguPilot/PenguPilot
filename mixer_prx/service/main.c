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
  
 Mixer Input Proxy Implementation

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <util.h>
#include <service.h>
#include <msgpack_reader.h>
#include <pp_prio.h>


MSGPACK_PROXY_DECL(torques)
MSGPACK_PROXY_DECL(thrust)


SERVICE_MAIN_BEGIN("mixer_prx", 0)
{
   MSGPACK_PROXY_START(torques, "torques_p", "pull", "torques", "pub", PP_PRIO_1);
   MSGPACK_PROXY_START(thrust, "thrust_p", "pull", "thrust", "pub", PP_PRIO_1);
   SERVICE_MAIN_PSEUDO_LOOP();
}
SERVICE_MAIN_END

