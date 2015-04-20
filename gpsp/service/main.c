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



#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <util.h>
#include <daemon.h>

#include "main_serial.h"
#include "main_i2c.h"

#include <opcd_interface.h>
#include <scl.h>
#include <syslog.h>

void _cleanup(void)
{

}


void _main(int argc, char *argv[])
{
   (void)argc;
   (void)argv;

   opcd_params_init("", 0);
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
}


int main(int argc, char *argv[])
{
   char pid_file[1024];
   service_name_to_pidfile(pid_file, "gpsp");
   daemonize(pid_file, _main, _cleanup, argc, argv);
   return 0;
}


