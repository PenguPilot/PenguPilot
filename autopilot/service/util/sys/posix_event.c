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
  
 File Purpose

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


/* 
 * File: posix_lock.h
 * Purpose: event implementation using POSIX
 *
 * Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#include "posix_event.h"


#include <pthread.h>
#include <time.h>
#include <malloc.h>


typedef struct
{
   pthread_mutex_t mutex;
   pthread_cond_t cv;
}
posix_event_t;


static void *posix_event_create(void)
{
   posix_event_t *pev = malloc(sizeof(posix_event_t));
   pthread_mutex_init(&pev->mutex, NULL);
   pthread_cond_init(&pev->cv, NULL);
   return pev;
}


static int posix_event_timed_wait(void *event, unsigned int timeout)
{
   posix_event_t *pev = (posix_event_t *)event;
   pthread_mutex_lock(&pev->mutex);
   struct timespec ts;
   clock_gettime(CLOCK_REALTIME, &ts);
   ts.tv_sec += timeout;
   int ret = pthread_cond_timedwait(&pev->cv, &pev->mutex, &ts);
   pthread_mutex_unlock(&pev->mutex);
   return ret;
}


static void posix_event_wait(void *event)
{
   posix_event_t *pev = (posix_event_t *)event;
   pthread_mutex_lock(&pev->mutex);
   pthread_cond_wait(&pev->cv, &pev->mutex);
   pthread_mutex_unlock(&pev->mutex);
}


static void posix_event_signal(void *event)
{
   posix_event_t *pev = (posix_event_t *)event;
   pthread_mutex_lock(&pev->mutex);
   pthread_cond_broadcast(&pev->cv);
   pthread_mutex_unlock(&pev->mutex);
}


void posix_event_interface_init(event_interface_t *interface)
{
   interface->create = posix_event_create;
   interface->timed_wait = posix_event_timed_wait;
   interface->wait = posix_event_wait;
   interface->signal = posix_event_signal;
}

