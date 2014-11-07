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
  
 Serial Port Interface
 
 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include <fcntl.h>
#include <linux/termios.h>


#ifndef __SERIAL_H__
#define __SERIAL_H__


typedef struct
{
   char *path;
   int handle;
}
serialport_t;


int serial_open(serialport_t *port, char *path, int baudrate, int rw_mode, int flags);

int serial_read_char(const serialport_t *port);

int serial_read_buffer(char *buffer, int buf_size, const serialport_t *port);

int serial_write(const serialport_t *port, const char *buffer, unsigned int len);

int serial_write_str(const serialport_t *port, const char *str);

int serial_close(serialport_t *port);


#endif /* __SERIAL_H__ */

