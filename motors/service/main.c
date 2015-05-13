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
  
 Motors Service

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 interfaces:
 -----------
                            _________
                voltage -> |         | <-> opcd
    [sum, c_0, .., c_n]    | MOTORS  | <-- flight_state: integer; 0 | 1
                 forces -> | SERVICE | --> motors_state: integer;
 [enable, f_0, .., f_n]    |_________|     2 | 4 | 5 | 6 | 7


 base states:
 ------------
 0 = stopped
 1 = starting
 2 = running
 3 = stopping

 state chart:
 ------------

                 start()
         .-> [0] ---> [1] --.
 timer() |                  | timer()
         `-- [3] <--- [2] <-´
                 stop()


 extended states:
 ----------------
 2 = running, normal
 4 = stopped, saturated
 5 = starting, saturated
 6 = running, saturated
 7 = stopping, saturated

 state access macros:
 --------------------
 #define MOTORS_CONTROLLABLE(state) ((state) == 2 || (state) == 6)
 #define MOTORS_SATURATED(state) (((state) & 0x40) ? 1 : 0)


 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <syslog.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <math.h>

#include <service.h>
#include <daemon.h>
#include <logger.h>
#include <sched.h>
#include <opcd_interface.h>
#include <msgpack_reader.h>
#include <threadsafe_types.h>
#include <interval.h>

#include "motors_state_machine.h"
#include "force_to_esc/force_to_esc.h"
#include "drivers/arduino_pwms/arduino_pwms.h"


#define MIN_GAS 0.1f
#define SERVICE_NAME "motors"
#define SERVICE_PRIO 99



static struct sched_param sp;
static msgpack_sbuffer *msgpack_buf = NULL;
static char *platform = NULL;
static void *forces_socket = NULL;

static tsfloat_t voltage;


/* voltage reader thread: */
MSGPACK_READER_BEGIN(voltage_reader)
   MSGPACK_READER_LOOP_BEGIN(voltage_reader)
   if (root.type == MSGPACK_OBJECT_ARRAY)
      tsfloat_set(&voltage, root.via.array.ptr[0].via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


SERVICE_MAIN_BEGIN
{
   /* start voltage reader thread: */
   tsfloat_init(&voltage, 16.0);
   MSGPACK_READER_START(voltage_reader, "voltage", 99);
 
   /* initialize SCL: */
   forces_socket = scl_get_socket("motor_forces", "sub");
   THROW_IF(forces_socket == NULL, -EIO);

   /* init opcd: */
   char *platform;
   opcd_param_get("platform", &platform);
   syslog(LOG_INFO, "platform: %s", platform);
   
   /* initialize msgpack buffers: */
   msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);

   /* determine motor f2e: */
   char buffer_path[128];
   strcpy(buffer_path, platform);
   strcat(buffer_path, ".motors.f2e");
   char *f2e_name;
   opcd_param_get(buffer_path, &f2e_name);
   LOG(LL_INFO, "f2e: %s", f2e_name);
   float (*f2e)(float, float);
   if (strcmp(f2e_name, "mk12_roxxy282735_1045") == 0)
      f2e = f2e_mk12_roxxy282735_1045;
   else if (strcmp(f2e_name, "hk20_roxxy282735_1045") == 0)
      f2e = f2e_hk20_roxxy282735_1045;   
   else if (strcmp(f2e_name, "hexfet20_suppo221213_1045") == 0)
      f2e = f2e_hexfet20_suppo221213_1045;   
   else
      THROW(-EINVAL);
 
   /* determine number of motors: */
   strcpy(buffer_path, platform);
   strcat(buffer_path, ".motors.n_motors");
   int n_motors;
   opcd_param_get(buffer_path, &n_motors);
   LOG(LL_INFO, "number of motors: %d", n_motors);

   /* determine motor driver: */
   strcpy(buffer_path, platform);
   strcat(buffer_path, ".motors.driver");
   char *driver;
   opcd_param_get(buffer_path, &driver);
   LOG(LL_INFO, "driver: %s", driver);
   int (*write_motors)(float *forces);
   if (strcmp(driver, "arduino") == 0)
   {
      arduino_pwms_init();
      write_motors = arduino_pwms_write;   
   }
   motors_state_machine_init();
   interval_t interval;
   interval_init(&interval);

   while (running)
   {
      char buffer[1024];
      int ret = scl_recv_static(forces_socket, buffer, sizeof(buffer));
      if (ret > 0)
      {
         msgpack_unpacked msg;
         msgpack_unpacked_init(&msg);
         if (msgpack_unpack_next(&msg, buffer, ret, NULL))
         {
            /* read received raw channels message: */
            msgpack_object root = msg.data;
            assert (root.type == MSGPACK_OBJECT_ARRAY);
            int n_forces = root.via.array.size - 1;
            int enable = root.via.array.ptr[0].via.i64;
            float dt = interval_measure(&interval);
            motors_state_t state = motors_state_machine_update(dt, enable);
            float ctrls[16];
            FOR_N(i, n_forces)
            {
               float force = root.via.array.ptr[i + 1].via.dec;
               switch (state)
               {
                  case MOTORS_RUNNING:
                     ctrls[i] = fmax(MIN_GAS, f2e(force, tsfloat_get(&voltage)));
                     break;

                  case MOTORS_STARTING:
                     ctrls[i] = MIN_GAS;
                     break;

                  case MOTORS_STOPPED:
                  case MOTORS_STOPPING:
                     ctrls[i] = 0.0f;
               }
            }
            write_motors(ctrls);
         }
         msgpack_unpacked_destroy(&msg);
      }
      else
      {
         msleep(1);
      }
   }
}
SERVICE_MAIN_END

