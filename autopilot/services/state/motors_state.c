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
   MOTORS_SAFETY =   0x01,
   MOTORS_HALTED =   0x02,
   MOTORS_STOPPING = 0x04,
   MOTORS_STARTING = 0x08,
   MOTORS_SPINNING = 0x10,
   MOTORS_FLYING =   0x20
}
motors_state_t;


static motors_state_t state = MOTORS_SAFETY;
static motors_state_t prev_state = MOTORS_HALTED;


/* state names: */
static char *names[] =
{
   "SAFETY", 
   "HALTED", 
   "STOPPING", 
   "STOPPED", 
   "STARTING", 
   "SPINNING", 
   "FLYING"
};


static char *state_name(void)
{
   return names[((int)(logf(state)/logf(2)))];
}


static timer_t timer;
static float gas_start;
static float gas_stop;


/* initializes motor state */
void motors_state_init(float _gas_start, float _gas_stop)
{
   gas_start = _gas_start;
   gas_stop = _gas_stop;
   timer_init(&timer, 4.0);
}


/* indicates if the motors are safe */
int motors_state_safe(void)
{
   return state == MOTORS_SAFETY;
}


/* indicates if the motors are spinning */
int motors_state_spinning(void)
{
   return (state & (MOTORS_STARTING | MOTORS_SPINNING | MOTORS_FLYING)) ? 1 : 0;
}


/* indicates if the controller inputs are used  */
int motors_state_controllable(void)
{
   return (state & (MOTORS_SPINNING | MOTORS_FLYING)) ? 1 : 0;
}


void motors_state_update(flight_state_t flight_state, int lock, float gas, float dt, int start_allowed)
{
   switch (state)
   {
      case MOTORS_SAFETY:
         /* safety mode can be left by submitting a 0 lock value */
         if (!lock)
         {
            state = MOTORS_HALTED;   
         }
         break;

      case MOTORS_FLYING:
         /* in the FLYING state it is not allowed to spin down the motors:
            we have to go down first and go into the SPINNING state */
         if (flight_state == FS_STANDING)
         {
            state = MOTORS_SPINNING;   
         }
         break;

      case MOTORS_HALTED:
         /* in the halted state we can either lock the motors or we spin them up */
         if (lock)
         {
            state = MOTORS_SAFETY;   
         }
         else if (gas > gas_start && start_allowed)
         {
            state = MOTORS_STARTING;
            timer_reset(&timer);
         }
         break;
      
      case MOTORS_STARTING:
         /* in the starting state the motors spin up until minimum RPM;
            this state can be canceled by a low gas value */
         if (gas < gas_stop)
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
         /* spinning implies that we are standing on the ground;
            it is thus safe to spin down the motors */
         if (gas < gas_stop)
         {
            state = MOTORS_STOPPING;
            timer_reset(&timer);
         }
         else if (flight_state == FS_FLYING)
         {
            state = MOTORS_FLYING;   
         }
         break;
      
      case MOTORS_STOPPING:
         /* in the stopping state the motors spin down until they are stopped (0 RPM);
            this state can be canceled by a high gas value and starting condition */
         if (gas > gas_start && start_allowed)
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

