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
  
 MARG Calibration Service Implementation

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

#include <msgpack.h>
#include <daemon.h>
#include <util.h>
#include <scl.h>
#include <opcd_interface.h>
#include <serial.h>
#include <logger.h>
#include <marg_data.h>
#include <interval.h>

#include "calibration.h"
#include "acc_mag_cal.h"


static int running = 1;
static msgpack_sbuffer *msgpack_buf = NULL;
static msgpack_packer *pk = NULL;
static char *platform = NULL;
static void *marg_raw_socket = NULL;
static void *marg_cal_socket = NULL;
static calibration_t gyro_cal;
static interval_t gyro_move_interval;


int _main(void)
{
   THROW_BEGIN();
   char *name = "marg_cal";

   /* initialize msgpack buffers: */
   msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);
   pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   THROW_IF(pk == NULL, -ENOMEM);
  
   /* initialize SCL: */
   marg_raw_socket = scl_get_socket("marg_raw", "sub");
   THROW_IF(marg_raw_socket == NULL, -EIO);
   marg_cal_socket = scl_get_socket("marg_cal", "pub");
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
   cal_init(&gyro_cal, 3, 1000);
   acc_mag_cal_init();
   interval_init(&gyro_move_interval);
  
   while (1)
   {
      char buffer[1024];
      int ret = scl_recv_static(marg_raw_socket, buffer, sizeof(buffer));
      if (ret > 0)
      {
         msgpack_unpacked msg;
         msgpack_unpacked_init(&msg);
         marg_data_t marg_data;
         marg_data_init(&marg_data);
         if (msgpack_unpack_next(&msg, buffer, ret, NULL))
         {
            msgpack_object root = msg.data;
            assert (root.type == MSGPACK_OBJECT_ARRAY);
            uint8_t status = root.via.array.ptr[0].via.i64;
            FOR_N(i, 3)
               marg_data.gyro.ve[i] = root.via.array.ptr[1 + i].via.dec;
            FOR_N(i, 3)
               marg_data.acc.ve[i] = root.via.array.ptr[4 + i].via.dec;
            FOR_N(i, 3)
               marg_data.mag.ve[i] = root.via.array.ptr[7 + i].via.dec;
            float ultra_z = root.via.array.ptr[10].via.dec;
            float baro_z = root.via.array.ptr[11].via.dec;

            if (!cal_sample_apply(&gyro_cal, marg_data.gyro.ve))
            {
               if (gyro_moved(&marg_data.gyro))
               {
                  if (interval_measure(&gyro_move_interval) > 1.0)
                     LOG(LL_ERROR, "gyro moved during calibration, retrying");
                  cal_reset(&gyro_cal);
               }

            }
            else
            {
               ONCE(LOG(LL_INFO, "gyro biases: %f %f %f", gyro_cal.bias[0], gyro_cal.bias[1], gyro_cal.bias[2]));
            }

            acc_mag_cal_apply(&marg_data.acc, &marg_data.mag);

            msgpack_sbuffer_clear(msgpack_buf);
            msgpack_pack_array(pk, 12);
            PACKI(status);      /*  0 */
            PACKFV(marg_data.gyro.ve, 3); /* 1 - 3 */
            PACKFV(marg_data.acc.ve, 3);  /* 4 - 6 */
            PACKFV(marg_data.mag.ve, 3);  /* 7 - 9 */
            PACKF(ultra_z);     /* 10 */
            PACKF(baro_z);      /* 11 */
            scl_copy_send_dynamic(marg_cal_socket, msgpack_buf->data, msgpack_buf->size);
         }
         msgpack_unpacked_destroy(&msg);
      }
      else
      {
         msleep(1);
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
   sprintf(pid_file, "%s/.PenguPilot/run/marg_cal.pid", getenv("HOME"));
   daemonize(pid_file, main_wrap, _cleanup, argc, argv);
   return 0;
}

