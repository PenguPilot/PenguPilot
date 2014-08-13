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
  
 SCL Logger Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <stdarg.h>
#include <stdio.h>
#include <malloc.h>
#include <zmq.h>
#include <syslog.h>

#include <scl.h>
#include <threadsafe_types.h>
#include <opcd_interface.h>
#include <log_data.pb-c.h>

#include "logger.h"
#include "util.h"


static void *log_socket = NULL;
static tsint_t loglevel;
static tsint_t details;


int logger_open(void)
{
   ASSERT_ONCE();
   
   opcd_param_t params[] =
   {
      {"level", &loglevel},
      {"details", &details},
      OPCD_PARAMS_END
   };
   opcd_params_apply("logger.", params);
   
   log_socket = scl_get_socket("log_data");
   if (log_socket == NULL)
   {
      return -1;
   }
   return 0;
}


void logger_write(char *file, loglevel_t level, unsigned int line, char *format, ...)
{
   ASSERT_NOT_NULL(log_socket);
   ASSERT_NOT_NULL(file);

   if (level <= (unsigned int)tsint_get(&loglevel))
   {
      LogData log_data = LOG_DATA__INIT;

      /* fill log_data scalars #1: */
      log_data.level = level;
      log_data.file = file;
      log_data.line = line;
      log_data.details = (unsigned int)tsint_get(&details);

      /* set-up buffer for varg-message: */
      char message_buffer[1024];
      log_data.msg = message_buffer;

      /* fill log_data scalars #2: */
      va_list ap;
      va_start(ap, format);
      vsnprintf(message_buffer, sizeof(message_buffer), format, ap);
      va_end(ap);

      /* publish: */
      unsigned int log_data_len = (unsigned int)log_data__get_packed_size(&log_data);
      void *buffer = malloc(log_data_len);
      if (buffer != NULL)
      {
         log_data__pack(&log_data, buffer);
         scl_send_dynamic(log_socket, buffer, log_data_len, ZMQ_NOBLOCK);
      }
      else
      {
         syslog(LOG_CRIT, "malloc() failed in module logger");
      }
   }
}


int logger_close(void)
{
   ASSERT_NOT_NULL(log_socket);
   return zmq_close(log_socket);
}

