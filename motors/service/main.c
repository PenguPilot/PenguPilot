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
             voltage -> |         |
       motors_enable -> |         | <-> opcd
 [sum, c_0, .., c_n]    | MOTORS  | <-- flight_state: integer; 0 | 1
              forces -> | SERVICE | --> motors_state: integer;
      [f_0, .., f_n]    |_________|     2 | 4 | 5 | 6 | 7


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


#include <math.h>
#include <stdbool.h>

#include <service.h>
#include <logger.h>
#include <opcd_interface.h>
#include <msgpack_reader.h>
#include <threadsafe_types.h>
#include <interval.h>
#include <motors.h>
#include <pp_prio.h>

#include "motors_order_parser.h"
#include "motors_state_machine.h"
#include "force_to_esc/force_to_esc.h"
#include "drivers/arduino_pwms/arduino_pwms.h"


#define MIN_GAS 0.1 /* 10% */
#define MAX_GAS 0.8 /* 80% */


pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static enum
{
   MOTORS_OFF,
   MOTORS_NORMAL,
   MOTORS_TEST
} 
mot_en_mode = MOTORS_OFF;
static int mot_en_state[MAX_MOTORS];


/* mot_en reader thread: */
MSGPACK_READER_BEGIN(mot_en_reader)
   MSGPACK_READER_LOOP_BEGIN(mot_en_reader)
      pthread_mutex_lock(&mutex);
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         /* test code: */
         int n_motors = root.via.array.size;
         if (n_motors <= MAX_MOTORS)
         {
            FOR_N(i, n_motors)
            if (root.via.array.ptr[i].type == MSGPACK_OBJECT_POSITIVE_INTEGER)
            {
               mot_en_state[i] = root.via.array.ptr[i].via.i64;
            }
            mot_en_mode = MOTORS_TEST;
         }
      }
      else if (root.type == MSGPACK_OBJECT_POSITIVE_INTEGER)
      {
         /* normal mode: */
         mot_en_mode = root.via.i64 ? MOTORS_NORMAL : MOTORS_OFF;
      }
      pthread_mutex_unlock(&mutex);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


/* voltage reader thread: */
static tsfloat_t voltage;
MSGPACK_READER_BEGIN(voltage_reader)
   tsfloat_init(&voltage, 16.0f);
   MSGPACK_READER_LOOP_BEGIN(voltage_reader)
   if (root.type == MSGPACK_OBJECT_ARRAY)
      tsfloat_set(&voltage, root.via.array.ptr[0].via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


SERVICE_MAIN_BEGIN("motors", PP_PRIO_1)
{
   /* start voltage reader thread: */
   tsfloat_init(&voltage, 16.0);
   MSGPACK_READER_START(mot_en_reader, "mot_en", PP_PRIO_2, "pull");
   MSGPACK_READER_START(voltage_reader, "voltage", PP_PRIO_2, "sub");
 
   /* initialize SCL: */
   void *forces_socket = scl_get_socket("forces", "sub");
   THROW_IF(forces_socket == NULL, -EIO);
   void *int_en_socket = scl_get_socket("int_en", "pub");
   THROW_IF(int_en_socket == NULL, -EIO);
   void *int_reset_socket = scl_get_socket("int_reset", "pub");
   THROW_IF(int_reset_socket == NULL, -EIO);

   /* init opcd: */
   char *platform;
   opcd_param_get("platform", &platform);
   LOG(LL_INFO, "platform: %s", platform);
   
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
   
   /* determine motors order: */
   int order[MAX_MOTORS];
   strcpy(buffer_path, platform);
   strcat(buffer_path, ".motors.order");
   char *order_string;
   opcd_param_get(buffer_path, &order_string);
   int n_motors = motors_order_parser_run(order_string, order);

   /* determine motor driver: */
   strcpy(buffer_path, platform);
   strcat(buffer_path, ".motors.driver");
   char *driver;
   opcd_param_get(buffer_path, &driver);
   LOG(LL_INFO, "driver: %s", driver);
   int (*write_motors)(float *);
   if (strcmp(driver, "arduino") == 0)
   {
      arduino_pwms_init();
      write_motors = arduino_pwms_write;   
   }
   motors_state_machine_init();
   interval_t interval;
   interval_init(&interval);

   MSGPACK_PACKER_DECL_INFUNC();

   MSGPACK_READER_SIMPLE_LOOP_BEGIN(forces)
   {
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         int int_en = 0;
         int int_res = 1;

         int n_forces = root.via.array.size;
         float dt = interval_measure(&interval);
         pthread_mutex_lock(&mutex);
         motors_state_t state = motors_state_machine_update(dt, mot_en_mode);
         float ctrls[MAX_MOTORS];
         FOR_N(i, n_forces)
         {
            float force = root.via.array.ptr[i].via.dec;
            switch (state)
            {
               case MOTORS_RUNNING:
               {
                  float mot_gas = f2e(force, tsfloat_get(&voltage));
                  if (mot_gas < MIN_GAS)
                     mot_gas = MIN_GAS;
                  else if (mot_gas > MAX_GAS)
                     mot_gas = MAX_GAS;
                  else
                  {
                     int_en = 1;   
                  }
                  ctrls[order[i]] = mot_gas;
                  int_res = 0;
                  break;
               }

               case MOTORS_STARTING:
                  ctrls[order[i]] = MIN_GAS;
                  break;

               case MOTORS_STOPPED:
               case MOTORS_STOPPING:
                  ctrls[order[i]] = 0.0f;
            }
            if (mot_en_mode == MOTORS_TEST && !mot_en_state[i])
               ctrls[order[i]] = 0.0f;
         }
         pthread_mutex_unlock(&mutex);
          
         static int int_en_prev = 0;
         if (int_en_prev != int_en)
            int_en_prev = int_en;
 
         static int int_res_prev = 1;
         if (int_res_prev != int_res)
            int_res_prev = int_res;
         
         msgpack_sbuffer_clear(msgpack_buf);
         PACKI(int_en);
         scl_copy_send_dynamic(int_en_socket, msgpack_buf->data, msgpack_buf->size);

         msgpack_sbuffer_clear(msgpack_buf);
         PACKI(int_res);
         scl_copy_send_dynamic(int_reset_socket, msgpack_buf->data, msgpack_buf->size);

         write_motors(ctrls);
      }
   }
   MSGPACK_READER_SIMPLE_LOOP_END
}
SERVICE_MAIN_END

