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
  
 Local Time Interface

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef LTIME_H
#define LTIME_H


#include <time.h>
#include <sys/time.h>


#define NSEC_PER_MSEC 1000000L
#define USEC_PER_SEC  1000000L
#define NSEC_PER_SEC  1000000000L


struct timespec timespec_add_s(struct timespec ts, unsigned int s);

struct timespec timespec_add_ms(struct timespec ts, unsigned int ms);

struct timespec timespec_add(struct timespec a, struct timespec b);

#define TIMESPEC_ADD(dst, src, val) \
   do \
   { \
      (dst).tv_sec = (src).tv_sec + (val).tv_sec; \
      (dst).tv_nsec = (src).tv_nsec + (val).tv_nsec; \
      if ((dst).tv_nsec >= 1000000000) \
      { \
         (dst).tv_sec++; \
         (dst).tv_nsec -= 1000000000; \
      } \
   } \
   while (0)


#define TIMESPEC_SUB(dst, src, val) \
   do \
   { \
      (dst).tv_sec = (src).tv_sec - (val).tv_sec; \
      (dst).tv_nsec = (src).tv_nsec - (val).tv_nsec; \
      if ((dst).tv_nsec < 0) { \
         (dst).tv_sec--; \
         (dst).tv_nsec += 1000000000; \
      } \
   } \
   while (0)

int timespec_cmp(struct timespec *a, struct timespec *b);

#endif /* LTIME_H */

