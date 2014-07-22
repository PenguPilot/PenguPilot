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
  
 
 Magnetic Declination SCL Subscriber Implementation
 
 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <unistd.h>
#include <simple_thread.h>
#include <string.h>
#include <util.h>
#include <scl.h>

#include "scl_mag_decl.h"


#define THREAD_PRIORITY 92


static simple_thread_t thread;
static pthread_mutexattr_t mutexattr;
static pthread_mutex_t mutex;
static void *decl_socket;
static float decl;


SIMPLE_THREAD_BEGIN(thread_func)
{
   SIMPLE_THREAD_LOOP_BEGIN
   {
      char buffer[128];
      int len = scl_recv_static(decl_socket, buffer, sizeof(buffer));
      if (len > 0)
      {
         buffer[len] = '\0';
         pthread_mutex_lock(&mutex);
         sscanf(buffer, "%f", &decl);
         pthread_mutex_unlock(&mutex);
      }
      else
         sleep(1);
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END


int scl_mag_decl_init(void)
{
   decl_socket = scl_get_socket("decl");
   if (decl_socket == NULL)
   {
      return -1;
   }
   pthread_mutexattr_init(&mutexattr);
   pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT);
   pthread_mutex_init(&mutex, &mutexattr);
   simple_thread_start(&thread, thread_func, "geomag", THREAD_PRIORITY, NULL);
   return 0;
}


float scl_mag_decl_get(void)
{
   pthread_mutex_lock(&mutex);
   float _decl = decl;
   pthread_mutex_unlock(&mutex);
   return _decl;
}

