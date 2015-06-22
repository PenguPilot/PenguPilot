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
  
 SCL Logger Interface

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


/*
 * logger interface
 */

#ifndef __LOGGER_H__
#define __LOGGER_H__


#include <string.h>
#include <stdio.h>


typedef enum
{
   LL_ERROR,
   LL_WARNING,
   LL_INFO,
   LL_DEBUG,

   __LL_COUNT /* don't use me! */
}
loglevel_t;


/*
 * log message "printf"-like using a cetain debug level
 */
#define LOG(level, format, ...) do {  logger_write(__FILE__, level, __LINE__, format, ## __VA_ARGS__); } while (0)


/*
 * opens the logger, sets up connection using SCL
 */
int logger_open(const char *name);


/*
 * writes format string to logger with given log level
 */
void logger_write(char *file, loglevel_t level, unsigned int line, char *format, ...);


/*
 * closes the SCL logger connection
 */
int logger_close(void);


#endif /* __LOGGER_H__ */

