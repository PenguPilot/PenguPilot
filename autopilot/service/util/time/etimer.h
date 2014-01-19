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
  
 Expiring Timer Interface

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __ETIMER_H__
#define __ETIMER_H__


/* timer structure */
typedef struct
{
   float expire;
   float state;
}
etimer_t;

/* initializes a timer to expire in a given amout of time in seconds */
void etimer_init(etimer_t *timer, float expire);


/* resets the timer  */
void etimer_reset(etimer_t *timer);


/* advances the timer by dt seconds
   returns 1 if the timer expired and 0 if not */
int etimer_check(etimer_t *timer, float dt);


#endif /* __ETIMER_H__ */

