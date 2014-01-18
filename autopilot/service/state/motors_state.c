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
  
 Motors State Tracking

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <math.h>

#include "../util/time/timer.h"
#include "motors_state.h"


/* motor states: */
typedef enum 
{
   MOTORS_HALTED =   0x01,
   MOTORS_STOPPING = 0x02,
   MOTORS_STARTING = 0x04,
   MOTORS_SPINNING = 0x08
}
motors_state_t;


static motors_state_t state = MOTORS_HALTED;
static motors_state_t prev_state = MOTORS_HALTED;


/* state names: */
static char *names[] =
{
   "HALTED", 
   "STOPPING", 
   "STOPPED", 
   "STARTING", 
   "SPINNING",
};


static char *state_name(void)
{
   return names[((int)(logf(state)/logf(2)))];
}


static timer_t timer;


/* initializes motor state */
void motors_state_init(void)
{
   timer_init(&timer, 4.0);
}


/* indicates if the motors are spinning */
int motors_starting(void)
{
   return (state & MOTORS_STARTING) ? 1 : 0;
}


/* indicates if the motors are spinning */
int motors_spinning(void)
{
   return (state & (MOTORS_STARTING | MOTORS_SPINNING)) ? 1 : 0;
}


/* indicates if the controller inputs are used  */
int motors_controllable(void)
{
   return (state & MOTORS_SPINNING) ? 1 : 0;
}


void motors_state_update(flight_state_t flight_state, float dt, int start)
{
   switch (state)
   {
      case MOTORS_HALTED:
         if (start)
         {
            state = MOTORS_STARTING;
            timer_reset(&timer);
         }
         break;
      
      case MOTORS_STARTING:
         if (!start)
         {
            state = MOTORS_STOPPING;
            timer_reset(&timer);
         }
         else if (timer_check(&timer, dt))
         {
            state = MOTORS_SPINNING;
         }
         break;
      
      case MOTORS_SPINNING:
         if (!start && flight_state != FS_FLYING)
         {
            state = MOTORS_STOPPING;
            timer_reset(&timer);
         }
         break;
      
      case MOTORS_STOPPING:
         if (start)
         {
            state = MOTORS_STARTING;
            timer_reset(&timer);
         }
         else if (timer_check(&timer, dt))
         {
            state = MOTORS_HALTED;
            timer_reset(&timer);
         }
         break;
   }
}

