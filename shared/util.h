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
  
 Generic Util Functionality

 Copyright (C) 2010 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __UTIL_H__
#define __UTIL_H__


#include <time.h>
#include <sys/time.h>
#include <assert.h>
#include <stdlib.h>
#include <pthread.h>

#include <time.h>
#include <sys/time.h>


void user_data_dir(char *buffer);


#define PITCH 0
#define ROLL  1
#define YAW   2
#define GAS   3


/* do something only once (e.g. in a loop) */
#define ONCE(stmt) \
   do { \
      static int executed = 0; \
      if (executed == 0) \
      { \
         executed = 1; \
         stmt; \
      } \
   } while (0)


/* array length */
#define ARRAY_SIZE(array) \
   (sizeof(array) / sizeof(array[0]))


/* iterate through an array of known size */
#define FOR_EACH(i, array) \
   for (size_t i = 0; i < ARRAY_SIZE(array); i++)

/* iterate through [0, .., n - 1] using i */
#define FOR_N(i, n) \
   for (size_t i = 0; i < (size_t)n; i++)


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


#define ASSERT_ONCE() \
{ \
static pthread_mutex_t __mutex__ = PTHREAD_MUTEX_INITIALIZER; \
\
pthread_mutex_lock(&__mutex__); \
static int __done__ = 0; \
if (!__done__) \
__done__ = 1; \
else \
assert(!"fatal: code called more than once!"); \
pthread_mutex_unlock(&__mutex__); \
}


#define ASSERT_NULL(ptr_stmt) \
assert((ptr_stmt) == NULL)

#define ASSERT_NOT_NULL(ptr_stmt) \
assert((ptr_stmt) != NULL)

#define ASSERT_TRUE(stmt) \
assert(stmt)

#define ASSERT_FALSE(stmt) \
assert(!(stmt))

#define ASSERT_UNREACHABLE() \
assert(!"unreachable code executed!")


typedef struct
{
   void *data;
   size_t size;
}
binary_data_t;

typedef struct
{
   struct timespec now;
   struct timespec next;
   struct timespec period;
}
period_context_t;


typedef struct
{
   char *name;
   period_context_t context;
}
periodic_thread_data_t;


#define EVERY_N_TIMES(time, statement) \
   { \
      static int __c = 0; \
      if (!(__c++ % time)) \
      { \
         statement; \
      } \
   }


int compare_floats(const void *a, const void *b);

void delay_execution(unsigned int s, unsigned int ns);

void msleep(unsigned int ms);

float limit(float val, float min_val, float max_val);

float sym_limit(float val, float max_val);

int binsearch(int x, const int v[], int n);


#include <stdio.h>
#include <stddef.h>
#include <errno.h>


#define THROW_PROPAGATE(err) \
   if (err < 0) \
   { \
      fprintf(stderr, "error in file %s line %d: %d, errno: %d\n", __FILE__, __LINE__, err, errno); \
   } \
   return err;

#define THROW_ON_ERR(err) \
   ___return_code = err; \
   if (___return_code < 0) \
   { \
      fprintf(stderr, "error in file %s line %d: %d, errno: %d\n", __FILE__, __LINE__, ___return_code, errno); \
      goto __catch_label; \
   }

#define THROW_ON_ERR_EXCLUDE(err, code) \
   ___return_code = err; \
   if ((___return_code < 0) && (___return_code != code)) \
   { \
      fprintf(stderr, "error in file %s line %d: %d, errno: %d\n", __FILE__, __LINE__, ___return_code, errno); \
      goto __catch_label; \
   }
   
#define THROW_PREV \
   (___return_code)

#define THROW_IF(cond, code) \
   if (cond) \
   { \
      ___return_code = code; \
      goto __catch_label; \
   }

#define THROW_BEGIN() \
   int ___return_code = 0;

#define THROW_END() \
   __catch_label: \
   return ___return_code;

#define THROW_END_EXEC(cmd) \
   __catch_label: \
   cmd; \
   return ___return_code;


#endif /* __UTIL_H__ */
