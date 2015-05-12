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
  
 Timer-based Motors State Machine Implementation

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <util.h>
#include <math.h>
#include <logger.h>

#include <time/etimer.h>

#include "motors_state_machine.h"


static motors_state_t state = MOTORS_STOPPED;
static etimer_t timer;


void motors_state_machine_init(void)
{
   ASSERT_ONCE();
   etimer_init(&timer, 1.5);
}
void motors_state_machine_init(void);


motors_state_t motors_state_machine_update(float dt, bool start)
{
   switch (state)
   {
      case MOTORS_STOPPED:
         if (start)
         {
            state = MOTORS_STARTING;
            etimer_reset(&timer);
            LOG(LL_DEBUG, "MOTORS_STARTING");
         }
         break;
      
      case MOTORS_STARTING:
         if (etimer_check(&timer, dt))
         {
            state = MOTORS_RUNNING;
            LOG(LL_DEBUG, "MOTORS_RUNNING");
         }
         break;
      
      case MOTORS_RUNNING:
         if (!start)
         {
            state = MOTORS_STOPPING;
            etimer_reset(&timer);
            LOG(LL_DEBUG, "MOTORS_STOPPING");
         }
         break;
      
      case MOTORS_STOPPING:
         if (etimer_check(&timer, dt))
         {
            state = MOTORS_STOPPED;
            LOG(LL_DEBUG, "MOTORS_STOPPED");
         }
         break;
   }
   return state;
}

