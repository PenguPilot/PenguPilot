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
  
 MAG ADC Calibration Service Implementation

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <syslog.h>
#include <sched.h>
#include <unistd.h>

#include <msgpack.h>
#include <daemon.h>
#include <util.h>
#include <scl.h>
#include <opcd_interface.h>
#include <logger.h>

#include "mag_adc_cal.h"


static int running = 1;
static msgpack_sbuffer *msgpack_buf = NULL;
static msgpack_packer *pk = NULL;
static char *platform = NULL;
static void *marg_raw_socket = NULL;
static void *marg_cal_socket = NULL;
static char *name = "mag_adc_cal";


int _main(void)
{
   THROW_BEGIN();

   struct sched_param sp;
   sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
   sched_setscheduler(getpid(), SCHED_FIFO, &sp);
 
   /* initialize msgpack buffers: */
   msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);
   pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   THROW_IF(pk == NULL, -ENOMEM);
  
   /* initialize SCL: */
   marg_raw_socket = scl_get_socket("mag_raw", "sub");
   THROW_IF(marg_raw_socket == NULL, -EIO);
   marg_cal_socket = scl_get_socket("mag_adc_cal", "pub");
   THROW_IF(marg_cal_socket == NULL, -EIO);

   /* initialize logger: */
   syslog(LOG_INFO, "opening logger");
   if (logger_open(name) != 0)
   {  
      syslog(LOG_CRIT, "could not open logger");
      THROW(-EIO);
   }
   syslog(LOG_CRIT, "logger opened");
   
   /* init opcd: */
   opcd_params_init(name, 1);
   
   /* init calibration data: */
   mag_adc_cal_init();
 
   while (1)
   {
      char buffer[1024];
      int ret = scl_recv_static(marg_raw_socket, buffer, sizeof(buffer));
      if (ret > 0)
      {
         msgpack_unpacked msg;
         msgpack_unpacked_init(&msg);
         if (msgpack_unpack_next(&msg, buffer, ret, NULL))
         {
            msgpack_object root = msg.data;
            if (root.type == MSGPACK_OBJECT_ARRAY)
            {
               vec3_t mag;
               vec3_init(&mag);
               FOR_N(i, 3)
                  mag.ve[i] = root.via.array.ptr[i].via.dec;
               mag_adc_cal_apply(&mag);
               msgpack_sbuffer_clear(msgpack_buf);
               msgpack_pack_array(pk, 3);
               PACKFV(mag.ve, 3);
               scl_copy_send_dynamic(marg_cal_socket, msgpack_buf->data, msgpack_buf->size);
            }
         }
         msgpack_unpacked_destroy(&msg);
      }
      else
      {
         msleep(10);
      }
   }
  
   THROW_END();
}


void _cleanup(void)
{
   running = 0;
}


void main_wrap(int argc, char *argv[])
{
   (void)argc;
   (void)argv;

   exit(-_main());   
}


int main(int argc, char *argv[])
{
   char pid_file[1024];
   service_name_to_pidfile(pid_file, name);
   daemonize(pid_file, main_wrap, _cleanup, argc, argv);
   return 0;
}

