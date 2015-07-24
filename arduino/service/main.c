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


#include <msgpack.h>
#include <serial.h>
#include <scl.h>
#include <service.h>
#include <pp_prio.h>

#include "ppm_common.h"
#include "ppm_parse.h"
#include "power_common.h"
#include "power_parse.h"


#define CHANNELS_MAX (16)

//#undef SERVICE_MAIN_DEBUG
//#define SERVICE_MAIN_DEBUG 1
SERVICE_MAIN_BEGIN("arduino", PP_PRIO_2)
{
   /* init scl and get sockets:: */
   void *rc_socket = scl_get_socket("rc_raw", "pub");
   THROW_IF(rc_socket == NULL, -ENODEV);
   void *voltage_socket = scl_get_socket("voltage", "pub");
   THROW_IF(voltage_socket == NULL, -ENODEV);
   void *current_socket = scl_get_socket("current", "pub");
   THROW_IF(current_socket == NULL, -ENODEV);

   /* set-up msgpack packer: */
   MSGPACK_PACKER_DECL_INFUNC();
 
   /* fetch parameters: */
   LOG(LL_INFO, "reading parameters");
   char *dev_path;
   int dev_speed;
   opcd_param_get("u3_bitbang_arduino.arduino_serial.path", &dev_path);
   opcd_param_get("u3_bitbang_arduino.arduino_serial.speed", &dev_speed);
   
   /* open serial port: */
   LOG(LL_INFO, "opening serial port");
   serialport_t port;
   THROW_ON_ERR(serial_open(&port, dev_path, dev_speed, O_RDONLY, 0));

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
         
         /* send voltage: */
         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, 1);
         PACKF(voltage); /* index 0 */
	      scl_copy_send_dynamic(voltage_socket, msgpack_buf->data, msgpack_buf->size);
         
         /* send current: */
         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, 1);
         PACKF(current); /* index 0 */
	      scl_copy_send_dynamic(current_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
}
SERVICE_MAIN_END

