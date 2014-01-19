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
  
 Autopilot Service

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

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
#include <daemon.h>
#include <stdbool.h>

#include "main_loop/main_util.h"
#include "main_loop/main_realtime.h"
#include "main_loop/main_replay.h"


int main(int argc, char *argv[])
{
   char *file = NULL;
   if (argc > 1)
   {
      file = argv[1];
   }

   bool calibrate = false;
   if (file && strcmp(file, "calibrate") == 0)
      calibrate = true;

   if (!calibrate && file)
   {
      main_replay(argc, argv);
   }
   else
   {
      main_realtime(argc, argv);
      char pid_file[1024];
      sprintf(pid_file, "%s/.PenguPilot/run/autopilot.pid", getenv("HOME"));
      daemonize(pid_file, main_realtime, die, argc, argv);
   }
   return 0;
}

