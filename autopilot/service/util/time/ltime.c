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
  
 Local Time Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "ltime.h"


struct timespec timespec_add_s(struct timespec ts, unsigned int s)
{
   struct timespec delta = {s, 0};
   return timespec_add(ts, delta);
}

struct timespec timespec_add_ms(struct timespec ts, unsigned int ms)
{
   struct timespec delta = {0, NSEC_PER_MSEC * ms};
   return timespec_add(ts, delta);
}


struct timespec timespec_add(struct timespec a, struct timespec b)
{
   a.tv_sec += b.tv_sec;
   a.tv_nsec += b.tv_nsec;
   while (a.tv_nsec >= NSEC_PER_SEC)
   {
      a.tv_sec++;
      a.tv_nsec -= NSEC_PER_SEC;
   }
   return a;
}


/*
 * returns 0 if ta and tb are equal
 * returns -1 if ta < tb
 * returns 1 if ta > tb
 */
int timespec_cmp(struct timespec *ta, struct timespec *tb)
{
   if (ta == tb)
      return 0;
   if (ta->tv_sec < tb->tv_sec)
      return -1;
   if (ta->tv_sec > tb->tv_sec)
      return 1;
   if (ta->tv_nsec < tb->tv_nsec)
      return -1;
   if (ta->tv_nsec > tb->tv_nsec)
      return 1;
   return 0;
}


