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
  
 Gro Calibration Service Implementation

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
#include <math.h>
#include <stdbool.h>

#include <msgpack.h>
#include <daemon.h>
#include <util.h>
#include <scl.h>
#include <opcd_interface.h>
#include <serial.h>
#include <logger.h>
#include <interval.h>

#include "calibration.h"


static int running = 1;
static msgpack_sbuffer *msgpack_buf = NULL;
static msgpack_packer *pk = NULL;
static char *platform = NULL;
static void *gyro_raw_socket = NULL;
static void *gyro_cal_socket = NULL;
static calibration_t gyro_cal;
static interval_t gyro_move_interval;
static char *name = "gyro_cal";


static bool gyro_moved(const float *gyro)
{
   FOR_N(i, 3)
   {  
      if (fabs(gyro[i]) > 0.15)
      {  
         return 1;
      }
   }
   return 0;
}


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
   gyro_raw_socket = scl_get_socket("gyro_raw", "sub");
   THROW_IF(gyro_raw_socket == NULL, -EIO);
   gyro_cal_socket = scl_get_socket("gyro_cal", "pub");
   THROW_IF(gyro_cal_socket == NULL, -EIO);

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
   cal_init(&gyro_cal, 3, 1000);
   interval_init(&gyro_move_interval);
   

   while (1)
   {
      char buffer[1024];
      int ret = scl_recv_static(gyro_raw_socket, buffer, sizeof(buffer));
      if (ret > 0)
      {
         msgpack_unpacked msg;
         msgpack_unpacked_init(&msg);
         if (msgpack_unpack_next(&msg, buffer, ret, NULL))
         {
            float gyro[3];
            msgpack_object root = msg.data;
            if (root.type == MSGPACK_OBJECT_ARRAY)
            {
               FOR_N(i, 3)
                  gyro[i] = root.via.array.ptr[i].via.dec;

               if (!cal_sample_apply(&gyro_cal, gyro))
               {
                  if (gyro_moved(gyro))
                  {
                     if (interval_measure(&gyro_move_interval) > 1.0)
                         LOG(LL_ERROR, "gyro moved during calibration, retrying");
                     cal_reset(&gyro_cal);
                  }
               }
               else
               {
                  ONCE(LOG(LL_INFO, "gyro biases: %f %f %f", gyro_cal.bias[0], gyro_cal.bias[1], gyro_cal.bias[2]));
                  msgpack_sbuffer_clear(msgpack_buf);
                  msgpack_pack_array(pk, 3);
                  PACKFV(gyro, 3);
                  scl_copy_send_dynamic(gyro_cal_socket, msgpack_buf->data, msgpack_buf->size);
               }
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

