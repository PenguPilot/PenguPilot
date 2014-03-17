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
  
 Process Daemonizer with Pidfile Support
 Inspired by: http://www.netzmafia.de/skripten/unix/linux-daemon-howto.html
 Most of the code has been hacked by Devin Watson, thanks!
 
 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <sys/stat.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

#include "daemon.h"
#include "pidfile.h"


static main_func_t g_main = NULL;
static clean_func_t g_clean = NULL;
static char *g_pid_path = NULL;


static void term_handler(int sig)
{
   (void)sig;
   remove_pid(g_pid_path);
   g_clean();
   exit(EXIT_SUCCESS);
}


static void daemon_main(int argc, char *argv[])
{
   struct sigaction new, old;
   new.sa_handler = term_handler;
   sigemptyset(&new.sa_mask);
   new.sa_flags = 0;
   sigaction(SIGINT, &new, &old);
   sigaction(SIGTERM, &new, &old);
   write_pid(g_pid_path);
   g_main(argc, argv);
}


void daemonize(char *pid_path, main_func_t main, clean_func_t clean, int argc, char *argv[])
{
   g_pid_path = pid_path;
   g_main = main;
   g_clean = clean;

   if (check_pid(pid_path))
   {
      exit(EXIT_FAILURE);
   }

   pid_t pid, sid;
   pid = fork();
   if (pid < 0)
   {
      exit(EXIT_FAILURE);
   }
   
   if (pid > 0)
   {
      exit(EXIT_SUCCESS);
   }

   umask(0);
   sid = setsid();
   if (sid < 0)
   {
      exit(EXIT_FAILURE);
   }

   if ((chdir("/")) < 0)
   {
      exit(EXIT_FAILURE);
   }

   close(STDIN_FILENO);
   close(STDOUT_FILENO);
   close(STDERR_FILENO);
   open("/dev/null", O_RDONLY); /* stdin */
   open("/dev/null", O_WRONLY); /* stderr */
   open("/dev/null", O_RDWR); /* stdout */

   daemon_main(argc, argv);
   exit(EXIT_SUCCESS);
}

