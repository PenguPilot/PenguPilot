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
  
 S.Bus Serial Port Interface
 NOTE: requires logic inverter on UART receiver line

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */




#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <linux/termios.h>


#include "sbus_serial.h"


int ioctl(int d, int request, ...);


int sbus_serial_open(const char *device)
{
	int fd = open(device, O_RDWR | O_NOCTTY);
   if (fd < 0)
      return fd;
   
   struct termios2 tio;
   ioctl(fd, TCGETS2, &tio);
	tio.c_cflag = CS8 | CLOCAL | CREAD | PARENB | CSTOPB | BOTHER;
   tio.c_oflag = 0;
   tio.c_iflag = 0;
   tio.c_lflag = 0;

   tio.c_cc[VINTR]    = 0;     /* Ctrl-c */
   tio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
   tio.c_cc[VERASE]   = 0;     /* del */
   tio.c_cc[VKILL]    = 0;     /* @ */
   tio.c_cc[VEOF]     = 0;     /* Ctrl-d */
   tio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
   tio.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
   tio.c_cc[VSWTC]    = 0;     /* '\0' */
   tio.c_cc[VSTART]   = 0;     /* Ctrl-q */
   tio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
   tio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
   tio.c_cc[VEOL]     = 0;     /* '\0' */
   tio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
   tio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
   tio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
   tio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
   tio.c_cc[VEOL2]    = 0;     /* '\0' */

   int speed = 100000;
   tio.c_ispeed = speed;
   tio.c_ospeed = speed;
   ioctl(fd, TCSETS2, &tio);
   return fd;
}


int sbus_serial_read(int fd)
{
   uint8_t c;
   if (read(fd, &c, 1) < 0)
      return -EIO;
   return c;
}

