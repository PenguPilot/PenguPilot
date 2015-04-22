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
  
 I2C Sensor Reader / Publisher

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <syslog.h>
#include <stdlib.h>
#include <daemon.h>
#include <stdbool.h>
#include <msgpack.h>

#include <sched.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/mman.h>

#include <scl.h>
#include <opcd_interface.h>
#include <periodic_thread.h>
#include <util.h>
#include <interval.h>
#include <logger.h>

#include "platform/platform.h"
#include "platform/exynos_quad.h"
#include "platform/pi_quad.h"

#include "emitters/baro_emitter.h"
#include "emitters/ultra_emitter.h"
#include "emitters/mag_emitter.h"


#define REALTIME_PERIOD 0.005



static int _main(int argc, char *argv[])
{
   ASSERT_ONCE();
   THROW_BEGIN();

   struct sched_param sp;
   periodic_thread_t _thread; 
   periodic_thread_t *thread = &_thread;
   char *name = "i2c_sensors";
   char *plat_name;
   msgpack_sbuffer *msgpack_buf = NULL;
   msgpack_packer *pk = NULL;
 
   /* initialize SCL: */
   syslog(LOG_INFO, "initializing scl");
   void *gyro_raw_socket = scl_get_socket("gyro_raw", "pub");
   THROW_IF(gyro_raw_socket == NULL, -EIO);
   void *acc_raw_socket = scl_get_socket("acc_raw", "pub");
   THROW_IF(acc_raw_socket == NULL, -EIO);

   /* initialize msgpack buffers: */
   msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);
   pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   THROW_IF(pk == NULL, -ENOMEM);
 
   /* init params subsystem: */
   opcd_params_init(name, 1);

   /* initialize logger: */
   syslog(LOG_INFO, "opening logger");
   THROW_ON_ERR(logger_open(name));

   /* determine platform: */
   THROW_ON_ERR(opcd_param_get("platform", &plat_name));
   if (strcmp(plat_name, "pi_quad") == 0)
   {
      if (pi_quad_init(&platform) < 0)
      {
         LOG(LL_ERROR, "could not initialize platform");
      }
   }
   else if (strcmp(plat_name, "exynos_quad") == 0)
   {
      if (exynos_quad_init(&platform) < 0)
      {
         LOG(LL_ERROR, "could not initialize platform");
      }
   }
   else
   {
      LOG(LL_ERROR, "unknown platform: %s", plat_name);
   }

   LOG(LL_INFO, "setting up real-time scheduling");
   sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
   sched_setscheduler(getpid(), SCHED_FIFO, &sp);
   
   if (nice(-20) == -1)
   {
      LOG(LL_ERROR, "could not renice process");
   }
 
   if (platform.read_mag)
      mag_emitter_start();
   if (platform.read_baro)
      baro_emitter_start();
   if (platform.read_ultra)
      ultra_emitter_start();

   thread->sched_param.sched_priority = 99;
   pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread->sched_param);

   thread->name = "gyro_acc";
   thread->running = 1;
   thread->periodic_data.period.tv_sec = 0;
   thread->periodic_data.period.tv_nsec = NSEC_PER_SEC * REALTIME_PERIOD;

   PERIODIC_THREAD_LOOP_BEGIN
   {
      vec3_t gyro;
      vec3_init(&gyro);
      int ret = platform_read_gyro(&gyro);
      
      msgpack_sbuffer_clear(msgpack_buf);
      if (ret == 0)
      {
         msgpack_pack_array(pk, 3);
         PACKFV(gyro.ve, 3);
      }
      else
      {
         PACKI(ret);
      }
      scl_copy_send_dynamic(gyro_raw_socket, msgpack_buf->data, msgpack_buf->size);
      
      vec3_t acc;
      vec3_init(&acc);
      ret = platform_read_acc(&acc);
      msgpack_sbuffer_clear(msgpack_buf);
      if (ret == 0)
      {
         msgpack_pack_array(pk, 3);
         PACKFV(acc.ve, 3);
      }
      else
      {
         PACKI(ret);
      }
      scl_copy_send_dynamic(acc_raw_socket, msgpack_buf->data, msgpack_buf->size);
   }
   PERIODIC_THREAD_LOOP_END

   THROW_END();
}


void main_wrap(int argc, char *argv[])
{
   _main(argc, argv);  
}


void pp_daemonize(char *name, int argc, char *argv[])
{
   char pid_file[1024];
   service_name_to_pidfile(pid_file, name);
   daemonize(pid_file, main_wrap, die, argc, argv);
}


int main(int argc, char *argv[])
{
   pp_daemonize("i2c_sensors", argc, argv);
   return 0;
}

