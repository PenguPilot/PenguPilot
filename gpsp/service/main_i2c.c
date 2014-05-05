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
 
 I2C GPS Publisher

 Copyright (C) 2014 Martin Turetschek, Ilmenau University of Technology
 Copyright (C) 2014 Kevin Ernst, Ilmenau University of Technology
 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <syslog.h>
#include <math.h>

#include <util.h>
#include <threadsafe_types.h>
#include <opcd_interface.h>
#include <scl.h>
#include <i2c/i2c.h>
#include <msgpack.h>

#include "i2c/navigatron_gtpa010.h"


int main_i2c(void)
{
   THROW_BEGIN();

   void *gps_socket = scl_get_socket("gps");
   THROW_IF(gps_socket == NULL, -EIO);
   int64_t hwm = 1;
   zmq_setsockopt(gps_socket, ZMQ_SNDHWM, &hwm, sizeof(hwm));

   void *sats_socket = scl_get_socket("sats");
   THROW_IF(sats_socket == NULL, -EIO);
 
   i2c_bus_t bus;
   i2c_dev_t device;

   uint8_t data_w[128];
   uint8_t data_r[128];   

   if (i2c_bus_open(&bus, "/dev/i2c-1"))
   {
      syslog(LOG_CRIT, "could not open i2c device");   
      exit(EXIT_FAILURE);
   }
   i2c_dev_init(&device, &bus, I2C_GPS_ADDRESS);
   
   msgpack_sbuffer *msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);
   msgpack_packer *pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   THROW_IF(pk == NULL, -ENOMEM);

   while (1)
   {
      msleep(200);

      data_w[0] = I2C_GPS_STATUS;
      i2c_xfer(&device, 1, data_w, 1, data_r);
      
      msgpack_sbuffer_clear(msgpack_buf);
      int status = data_r[0];
      int fix = 0;
      if (status & I2C_GPS_STATUS_2DFIX)
         fix = 2;
      if (status & I2C_GPS_STATUS_3DFIX)
         fix = 3;

      if (fix == 2)
         msgpack_pack_array(pk, 7); /* 2d fix */
      else if (fix == 3)
         msgpack_pack_array(pk, 9); /* 3d fix */
      else
         msgpack_pack_array(pk, 1); /* no fix */
 
      data_w[0] = I2C_GPS_TIME;
      i2c_xfer(&device, 1, data_w, 4, data_r);
      long time = (((long) data_r[3]) << 24)
                | (((long) data_r[2]) << 16) 
                | (((long) data_r[1]) << 8) 
                | (data_r[0]);
      char *time_str = ctime(&time);
      size_t len = strlen(time_str);
      msgpack_pack_raw(pk, len);
      msgpack_pack_raw_body(pk, time_str, len); /* gps array index 0 */

      if (fix == 2 || fix == 3)
      {
         data_w[0] = I2C_GPS_LOCATION;
         i2c_xfer(&device, 1, data_w, 8, data_r);
         PACKF(( (((long) data_r[3]) << 24) 
               | (((long) data_r[2]) << 16) 
               | (((long) data_r[1]) << 8) 
               | (data_r[0])) / 10000000.0); /* latitude, gps array index 1 */

         PACKF(( (((long) data_r[7]) << 24) 
               | (((long) data_r[6]) << 16) 
               | (((long) data_r[5]) << 8) 
               | (data_r[4])) / 10000000.0); /* logitude, gps array index 2 */

         PACKI((status & I2C_GPS_STATUS_NUMSATS) >> 4); /* gps array index 3 */
         
         data_w[0] = I2C_GPS_GROUND_SPEED;
         i2c_xfer(&device, 1, data_w, 2, data_r);
         PACKF(((float)((data_r[1] << 8) | data_r[0])) / 100.0 * 1.94384); /* gps array index 4 */
         
         data_w[0] = I2C_GPS_GROUND_SPEED;
         i2c_xfer(&device, 1, data_w, 2, data_r);
         PACKF((data_r[1] << 8) | data_r[0]); /* gps array index 5 */
         
         PACKF(0 /* HDOP */); /* gps array index 6 */
      }

      if (fix == 3)
      {
         data_w[0] = I2C_GPS_ALTITUDE;
         THROW_ON_ERR(i2c_xfer(&device, 1, data_w, 2, data_r));
         PACKF(((data_r[1]) << 8) | data_r[0]);  /* gps array index 7 */
         PACKF(0 /* VDOP */); /* gps array index 8 */
      }
            
      scl_copy_send_dynamic(gps_socket, msgpack_buf->data, msgpack_buf->size);
   }
   THROW_END();
}

