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
 
 States:
 ------------
 0 = stopped
 1 = starting
 2 = running
 3 = stopping

 State chart:
 ------------

                 start()
         .-> [0] ---> [1] --.
 timer() |                  | timer()
         `-- [3] <--- [2] <-´
                 stop()


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
#include <scl.h>

#include <time/etimer.h>

#include "motors_state_machine.h"


static motors_state_t state = MOTORS_STOPPED;
static etimer_t timer;
MSGPACK_PACKER_DECL;
static void *motors_state_socket;


int motors_state_machine_init(void)
{
   ASSERT_ONCE();
   THROW_BEGIN();
   etimer_init(&timer, 1.5);
   MSGPACK_PACKER_INIT();
   motors_state_socket = scl_get_socket("motors_state", "pub");
   THROW_IF(motors_state_socket == NULL, -EIO);
   THROW_END();
}


static void publish_state_update(int new_state)
{
   msgpack_sbuffer_clear(msgpack_buf);
   PACKI(new_state);
   scl_copy_send_dynamic(motors_state_socket, msgpack_buf->data, msgpack_buf->size);
}


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
            publish_state_update(state);
         }
         break;
      
      case MOTORS_STARTING:
         if (etimer_check(&timer, dt))
         {
            state = MOTORS_RUNNING;
            LOG(LL_DEBUG, "MOTORS_RUNNING");
            publish_state_update(state);
         }
         break;
      
      case MOTORS_RUNNING:
         if (!start)
         {
            state = MOTORS_STOPPING;
            etimer_reset(&timer);
            LOG(LL_DEBUG, "MOTORS_STOPPING");
            publish_state_update(state);
         }
         break;
      
      case MOTORS_STOPPING:
         if (etimer_check(&timer, dt))
         {
            state = MOTORS_STOPPED;
            LOG(LL_DEBUG, "MOTORS_STOPPED");
            publish_state_update(state);
         }
         break;
   }
   return state;
}

