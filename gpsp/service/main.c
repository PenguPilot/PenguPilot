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
 
 GPS Publisher Program Entry Point

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include <stdlib.h>
#include <string.h>

#include <util.h>
#include <daemon.h>
#include <service.h>
#include <pp_prio.h>

#include "main_serial.h"
#include "main_i2c.h"



SERVICE_MAIN_BEGIN("gpsp", PP_PRIO_3)
   char *plat = NULL;
   opcd_param_get("platform", &plat);
   if (strcmp(plat, "overo_quad") == 0 || strcmp(plat, "exynos_quad") == 0)
   {
      main_serial();
   }
   else if (strcmp(plat, "pi_quad") == 0)
   {
      main_i2c();   
   }
SERVICE_MAIN_END

