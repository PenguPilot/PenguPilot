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
  
 Threadsafe Types
 
 declaration at module level:
    DECLARE_THREADSAFE_TYPE(float);
 
 declaration of individual variable:
    tsfloat f;
 
 initialization phase (once):
    tsfloat_init(&f);
 
 in writing thread:
    tsfloat_set(&f, 1.0);
 
 in reading thread:
    tsfloat_get(&f));

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __THREADSAFE_TYPES_H__
#define __THREADSAFE_TYPES_H__


#include <pthread.h>


#define DECLARE_THREADSAFE_TYPE(type) \
   \
   typedef struct \
   { \
      type value; \
      pthread_mutexattr_t mutexattr; \
      pthread_mutex_t mutex; \
   } \
   ts##type##_t; \
   \
   static inline void ts##type##_init(ts##type##_t *ts_var, type val) \
   { \
      pthread_mutexattr_init(&ts_var->mutexattr); \
      pthread_mutexattr_setprotocol(&ts_var->mutexattr, PTHREAD_PRIO_INHERIT); \
      pthread_mutex_init(&ts_var->mutex, &ts_var->mutexattr); \
      ts_var->value = val; \
   } \
   \
   static inline type ts##type##_get(ts##type##_t *ts_var) \
   { \
      type copy; \
      pthread_mutex_lock(&ts_var->mutex); \
      copy = ts_var->value; \
      pthread_mutex_unlock(&ts_var->mutex); \
      return copy; \
   } \
   \
   static inline void ts##type##_set(ts##type##_t *ts_var, type data) \
   { \
      pthread_mutex_lock(&ts_var->mutex); \
      ts_var->value = data; \
      pthread_mutex_unlock(&ts_var->mutex); \
   } \
   \
   static inline void ts##type##_copy(ts##type##_t *ts_var_a, ts##type##_t *ts_var_b) \
   { \
      pthread_mutex_lock(&ts_var_a->mutex); \
      pthread_mutex_lock(&ts_var_b->mutex); \
      ts_var_a->value = ts_var_b->value; \
      pthread_mutex_unlock(&ts_var_b->mutex); \
      pthread_mutex_unlock(&ts_var_a->mutex); \
   }


DECLARE_THREADSAFE_TYPE(float);
DECLARE_THREADSAFE_TYPE(int);


#endif /* __THREADSAFE_TYPES_H__ */

