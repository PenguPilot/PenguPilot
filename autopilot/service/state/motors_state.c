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

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <util.h>
#include <scl.h>
#include <math.h>

#include "../util/time/etimer.h"
#include "motors_state.h"


/* motors state: */
typedef enum 
{
   MOTORS_STOPPED =  0x01,
   MOTORS_STOPPING = 0x02,
   MOTORS_STARTING = 0x04,
   MOTORS_SPINNING = 0x08
}
motors_state_t;


static motors_state_t state = MOTORS_STOPPED;
static etimer_t timer;
static void *spinning_socket = NULL;


/* initializes motor state */
void motors_state_init(void)
{
   ASSERT_ONCE();
   ASSERT_NULL(spinning_socket);
   spinning_socket = scl_get_socket("motors_spinning");
   scl_send_static(spinning_socket, "false", 5);
   ASSERT_NOT_NULL(spinning_socket);
   etimer_init(&timer, 1.5);
}


/* indicates if the motors are starting */
bool motors_starting(void)
{
   return (state & MOTORS_STARTING) ? true : false;
}


/* indicates if the motors are stopping */
bool motors_stopping(void)
{
   return (state & MOTORS_STOPPING) ? true : false;
}


/* indicates if the motors are spinning */
bool motors_spinning(void)
{
   return (state & (MOTORS_STARTING | MOTORS_SPINNING | MOTORS_STOPPING)) ? true : false;
}


/* indicates if the controller inputs are used  */
bool motors_controllable(void)
{
   return (state & MOTORS_SPINNING) ? true : false;
}


void motors_state_update(bool flying, float dt, bool start)
{
   switch (state)
   {
      case MOTORS_STOPPED:
         if (start)
         {
            state = MOTORS_STARTING;
            etimer_reset(&timer);
            scl_send_static(spinning_socket, "true", 4);
         }
         break;
      
      case MOTORS_STARTING:
         if (etimer_check(&timer, dt))
         {
            state = MOTORS_SPINNING;
         }
         break;
      
      case MOTORS_SPINNING:
         if (!start && !flying)
         {
            state = MOTORS_STOPPING;
            etimer_reset(&timer);
         }
         break;
      
      case MOTORS_STOPPING:
         if (etimer_check(&timer, dt))
         {
            state = MOTORS_STOPPED;
            etimer_reset(&timer);
            scl_send_static(spinning_socket, "false", 5);
         }
         break;
   }
}

