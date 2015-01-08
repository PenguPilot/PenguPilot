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
  
 S.Bus Test Program

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <util.h>


#include "../shared/sbus_parser.h"
#include <serial.h>


int main(void)
{  
   THROW_BEGIN();

   /* open serial port: */
   serialport_t port;
   THROW_ON_ERR(serial_open(&port, "/dev/ttyO0", 100000, O_RDONLY, CSTOPB));
   
   /* init parser: */
   sbus_parser_t parser;
   sbus_parser_init(&parser);

   for (;;)
   {
      int b = serial_read_char(&port);
      if (sbus_parser_process(&parser, b))
         printf("%s | %.1f %.1f %.1f %.1f\n", parser.sig_valid ? "valid" : "invalid", parser.channels[0], parser.channels[1], parser.channels[2], parser.channels[3]);
   }
 
   THROW_END();
}

