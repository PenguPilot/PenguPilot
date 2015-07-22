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
  
 Motors Order Parser Implementation

 Copyright (C) 2015 Tobias Simon

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <stdio.h>
#include <assert.h>
#include <util.h>
#include <is_parser.h>

#include "mot_order.h"


void mot_order_print(int n_motors, int order[MAX_MOTORS])
{
   FOR_N(i, n_motors)
   {
      printf("%d\n", order[i]);   
   }
}


int mot_order_run(char *buffer, int order[MAX_MOTORS])
{
   is_parser_t parser;
   is_parser_init(&parser);
   unsigned int motors = is_parser_count(&parser, buffer);
   assert(motors < MAX_MOTORS);
   FOR_N(i, motors)
   {
       while (1)
       {
          is_parser_run(&parser, *buffer++);
          if (is_parser_ready(&parser))
             break;
       }
       order[i] = is_parser_val(&parser) - 1;
   }
   return motors;
}

