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
  
 Rotation Speed Control Proxy Implementation

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <msgpack.h>

#include <util.h>
#include <service.h>
#include <msgpack_reader.h>
#include <scl.h>
#include <pp_prio.h>


MSGPACK_PROXY_DECL(rs_ctrl_spp_p)
MSGPACK_PROXY_DECL(rs_ctrl_spp_r)
MSGPACK_PROXY_DECL(rs_ctrl_spp_y)


SERVICE_MAIN_BEGIN("rs_ctrl_prx", 0)
{
   MSGPACK_PROXY_START(rs_ctrl_spp_p, "rs_ctrl_spp_p", "pull", "rs_ctrl_sp_p", "pub", PP_PRIO_1);
   MSGPACK_PROXY_START(rs_ctrl_spp_r, "rs_ctrl_spp_r", "pull", "rs_ctrl_sp_r", "pub", PP_PRIO_1);
   MSGPACK_PROXY_START(rs_ctrl_spp_y, "rs_ctrl_spp_y", "pull", "rs_ctrl_sp_y", "pub", PP_PRIO_1);
   SERVICE_MAIN_PSEUDO_LOOP();
}
SERVICE_MAIN_END

