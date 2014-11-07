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

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <string.h>
#include <stdlib.h>
#include <util.h>


void user_data_dir(char *buffer)
{
   char *home = getenv("HOME");
   assert(home);
   sprintf(buffer, "%s/.PenguPilot", home);
}


int compare_floats(const void *a, const void *b)
{
   const float *da = (const float *) a;
   const float *db = (const float *) b;
   return (*da > *db) - (*da < *db);
}


void delay_execution(unsigned int s, unsigned int ns)
{
   struct timespec ts_wait;
   ts_wait.tv_sec = s;
   ts_wait.tv_nsec = ns;
   (void)clock_nanosleep(CLOCK_MONOTONIC, 0, &ts_wait, NULL);
}


void msleep(unsigned int ms)
{
   delay_execution(0, ms * NSEC_PER_MSEC);   
}


float limit(float val, float min_val, float max_val)
{
   float result;
   if (val > max_val)
   {
      result = max_val;
   }
   else if (val < min_val)
   {
      result = min_val;
   }
   else
   {
      result = val;
   }
   return result;
}


float sym_limit(float val, float max_val)
{
   return limit(val, -max_val, max_val);
}



int binsearch(int x, const int v[], int n)
{
   int low, high, mid;

   low = 0;
   high = n - 1;

   while (low <= high)
   {
      mid = (low + high) / 2;
      if (x < v[mid])
      {
         high = mid - 1;
      }
      else if ( x > v[mid])
      {
         low = mid + 1;
      }
      else /* found match */
         return mid;
   }
   return -1; /* no match */
}


void rel_val_init(rel_val_t *val)
{
   val->start = 0.0f;
   val->start_set = false;
}


float rel_val_get(rel_val_t *rel_val, float val)
{
   if (!rel_val->start_set)
   {
      rel_val->start = val;
      rel_val->start_set = true;
   }
   return val - rel_val->start;
}

