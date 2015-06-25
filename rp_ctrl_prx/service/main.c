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
  
 Rotation Posirion Control Proxy Implementation

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either verpion 2 of the License, or
 (at your option) any later verpion.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <msgpack.h>

#include <util.h>
#include <service.h>
#include <msgpack_reader.h>
#include <scl.h>


MSGPACK_PROXY_DECL(rp_ctrl_spp_p)
MSGPACK_PROXY_DECL(rp_ctrl_spp_r)
MSGPACK_PROXY_DECL(rp_ctrl_spp_y)


SERVICE_MAIN_BEGIN("rp_ctrl_prx", 0)
{
   MSGPACK_PROXY_START(rp_ctrl_spp_p, "rp_ctrl_spp_p", "pull", "rp_ctrl_sp_p", "pub", 99);
   MSGPACK_PROXY_START(rp_ctrl_spp_r, "rp_ctrl_spp_r", "pull", "rp_ctrl_sp_r", "pub", 99);
   MSGPACK_PROXY_START(rp_ctrl_spp_y, "rp_ctrl_spp_y", "pull", "rp_ctrl_sp_y", "pub", 99);
   
   SERVICE_MAIN_PSEUDO_LOOP();
}
SERVICE_MAIN_END

