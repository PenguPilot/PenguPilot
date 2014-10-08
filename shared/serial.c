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


#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <assert.h>

#include "serial.h"


static pthread_mutex_t write_mutex = PTHREAD_MUTEX_INITIALIZER;


int map_baudrate(int baudrate)
{
   int baudr;
   switch(baudrate)
   {
      case 50:
         baudr = B50;
         break;
       
      case 75:
         baudr = B75;
         break;
      
      case 110:
         baudr = B110;
         break;
      
      case 134:
         baudr = B134;
         break;

      case 150:
         baudr = B150;
         break;
      
      case 200:
         baudr = B200;
         break;
      
      case 300:
         baudr = B300;
         break;

      case 600: 
         baudr = B600;
         break;

      case 1200:
         baudr = B1200;
         break;

      case 1800:
         baudr = B1800;
         break;

      case 2400:
         baudr = B2400;
         break;

      case 4800: 
         baudr = B4800;
         break;

      case 9600:
         baudr = B9600;
         break;

      case 19200: 
         baudr = B19200;
         break;

      case 38400: 
         baudr = B38400;
         break;

      case 57600: 
         baudr = B57600;
         break;

      case 115200:
         baudr = B115200;
         break;

      case 230400: 
         baudr = B230400;
         break;

      case 460800:
         baudr = B460800;
         break;

      case 500000: 
         baudr = B500000;
         break;

      case 576000: 
         baudr = B576000;
         break;

      case 921600:
         baudr = B921600;
         break;

      case 1000000: 
         baudr = B1000000;
         break;

      default:
         assert(0);
   }
   return baudr;
}

int serial_open(serialport_t *port, const char *path, int baudrate, int rw_mode)
{
   struct termios new_options;
   port->path = path;
   port->handle = open(path, rw_mode | O_NOCTTY);
   if (port->handle == -1)
   {
      return -1;
   }
   else
   {
      (void)tcgetattr(port->handle, &port->orig_options);

      new_options.c_cflag = CREAD | CLOCAL | CS8;
      int baudr = map_baudrate(baudrate);
      cfsetospeed(&new_options, baudr);
      cfsetispeed(&new_options, baudr);
      new_options.c_oflag = 0;
      new_options.c_iflag = 0;
      new_options.c_lflag = 0;

      new_options.c_cc[VINTR]    = 0;     /* Ctrl-c */
      new_options.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
      new_options.c_cc[VERASE]   = 0;     /* del */
      new_options.c_cc[VKILL]    = 0;     /* @ */
      new_options.c_cc[VEOF]     = 0;     /* Ctrl-d */
      new_options.c_cc[VTIME]    = 0;     /* inter-character timer unused */
      new_options.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
      new_options.c_cc[VSWTC]    = 0;     /* '\0' */
      new_options.c_cc[VSTART]   = 0;     /* Ctrl-q */
      new_options.c_cc[VSTOP]    = 0;     /* Ctrl-s */
      new_options.c_cc[VSUSP]    = 0;     /* Ctrl-z */
      new_options.c_cc[VEOL]     = 0;     /* '\0' */
      new_options.c_cc[VREPRINT] = 0;     /* Ctrl-r */
      new_options.c_cc[VDISCARD] = 0;     /* Ctrl-u */
      new_options.c_cc[VWERASE]  = 0;     /* Ctrl-w */
      new_options.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
      new_options.c_cc[VEOL2]    = 0;     /* '\0' */

      (void)tcsetattr(port->handle, TCSANOW, &new_options);
      return 0;
   }
}


int serial_read_char(const serialport_t *port)
{
   unsigned char buffer;
   int ret;
   if ((ret = read(port->handle, &buffer, 1)) <= 0)
   {
      return -1;
   }
   return buffer;
}


int serial_read_buffer(char *buffer, int buf_size, const serialport_t *port)
{
   return read(port->handle, buffer, buf_size);
}



int serial_read_line(char buffer[256], const serialport_t *port)
{
   return read(port->handle, buffer, 256);
}


int serial_write(const serialport_t *port, const char *buffer, unsigned int len)
{
   int err;
   pthread_mutex_lock(&write_mutex);
   err = write(port->handle, buffer, len);
   pthread_mutex_unlock(&write_mutex);
   return err;
}


int serial_write_line(const serialport_t *port, const char *buffer)
{
   return serial_write(port, buffer, strlen(buffer));
}


int serial_close(serialport_t *port)
{
   (void)tcsetattr(port->handle, TCSANOW, &port->orig_options);
   return close(port->handle);
}

