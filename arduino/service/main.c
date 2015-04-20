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
  
 Arduino Remote Control and ADC Bridge Service

 Copyright (C) 2014 Jan Roemisch, Integrated Communication Systems Group, TU Ilmenau
 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <stdbool.h>
#include <msgpack.h>

#include <syslog.h>
#include <logger.h>
#include <util.h>
#include <serial.h>
#include <scl.h>
#include <opcd_interface.h>
#include <threadsafe_types.h>
#include <daemon.h>

#include "ppm_common.h"
#include "ppm_parse.h"
#include "power_common.h"
#include "power_parse.h"


#define CHANNELS_MAX (16)


static bool running = true;
static char *name = "arduino";


int _main(void)
{
   THROW_BEGIN()
   
   syslog(LOG_INFO, "opening logger");
   if (logger_open(name) != 0)
   {  
      syslog(LOG_CRIT, "could not open logger");
      THROW(-EIO);
   }
   syslog(LOG_CRIT, "logger opened");

   LOG(LL_INFO, "creating sockets");
   /* init scl and get sockets:: */
   void *rc_socket = scl_get_socket("rc_raw", "pub");
   THROW_IF(rc_socket == NULL, -ENODEV);
   void *power_socket = scl_get_socket("power", "pub");
   THROW_IF(power_socket == NULL, -ENODEV);

   /* allocate msgpack buffers: */
   msgpack_sbuffer *msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);
   msgpack_packer *pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   THROW_IF(pk == NULL, -ENOMEM);
 
   /* fetch parameters: */
   LOG(LL_INFO, "reading parameters");
   char *dev_path;
   tsint_t dev_speed;
   opcd_params_init("exynos_quad.arduino_serial.", 0);
   opcd_param_t params[] =
   {
      {"path", &dev_path},
      {"speed", &dev_speed},
      OPCD_PARAMS_END
   };
   opcd_params_apply("", params);
   
   /* open serial port: */
   LOG(LL_INFO, "opening serial port");
   serialport_t port;
   THROW_ON_ERR(serial_open(&port, dev_path, tsint_get(&dev_speed), O_RDONLY, 0));

   uint16_t channels_raw[PPM_CHAN_MAX];
   uint32_t voltage_raw, current_raw;
   float channels[CHANNELS_MAX];
   memset(channels, 0, sizeof(channels));

   while (running)
   {
      int c = serial_read_char(&port);
      if (c < 0)
         msleep(1);
      
      /* parse channels: */
      int status = ppm_parse_frame(channels_raw, (uint8_t)(c));
      if (status)
      {
         int sig_valid = 0;
         int invalid_count = 0;
         FOR_N(i, PPM_CHAN_MAX)
         {
            uint16_t chan_in = channels_raw[i];
            if (chan_in >= PPM_VALUE_INVALID)
               invalid_count++;
            if (chan_in > PPM_VALUE_MAX)
               chan_in = PPM_VALUE_MAX;
            if (chan_in < PPM_VALUE_MIN)
               chan_in = PPM_VALUE_MIN;
            channels[i] = (float)(chan_in - PPM_VALUE_MIN) / (PPM_VALUE_MAX - PPM_VALUE_MIN) * 2.f - 1.f;
         }
         sig_valid = (invalid_count < (PPM_CHAN_MAX / 3));
         
         /* send channels: */
         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, 1 + CHANNELS_MAX);
         PACKI(sig_valid);               /* index 0: valid */
         PACKFV(channels, CHANNELS_MAX); /* index 1, .. : channels */
         scl_copy_send_dynamic(rc_socket, msgpack_buf->data, msgpack_buf->size);
      }

      /* parse adc voltage/current: */
      status = power_parse_frame(&voltage_raw, &current_raw, (uint8_t)(c));
      if (status)
      {
         float voltage = (float)(voltage_raw) / 1000.0;
         float current = (float)(current_raw) / 1000.0;
         
         /* send voltage / current: */
         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, 2);
         PACKF(voltage); /* index 0 */
         PACKF(current); /* index 1 */
	      scl_copy_send_dynamic(power_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
   THROW_END();
}



void _cleanup(void)
{
   running = false;
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
   service_name_to_pidfile(pid_file, "arduino");
   daemonize(pid_file, main_wrap, _cleanup, argc, argv);
   return 0;
}

