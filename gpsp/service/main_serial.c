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
 
 Serial NMEA GPS Publisher and System Time Update Service

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <syslog.h>
#include <math.h>

#include <util.h>
#include <threadsafe_types.h>
#include <opcd_interface.h>
#include <serial.h>
#include <scl.h>
#include <msgpack.h>

#include "linux_sys.h"
#include "nmealib/nmea/info.h"
#include "nmealib/nmea/parser.h"


#define TIME_STR_LEN 128


static char running = 1;
 

static size_t generate_time_str(char str[TIME_STR_LEN], nmeaTIME *time)
{
   struct tm tm_time;
   tm_time.tm_year = time->year;
   tm_time.tm_mon = time->mon;
   tm_time.tm_mday = time->day;
   tm_time.tm_hour = time->hour;
   tm_time.tm_min = time->min;
   tm_time.tm_sec = time->sec;
   tm_time.tm_isdst = -1;

   if (mktime(&tm_time) == -1)
   {
      str[0] = '\0';
   }
   else
   {
      strftime(str, TIME_STR_LEN, "%Y-%m-%d %H:%M:%S", &tm_time);
   }
   return strlen(str);
}


static double convert(double val)
{
   double deg = ((int)(val / 100.0));
   val = deg + (val - deg * 100.0) / 60.0;
   return val;
}


int main_serial(void)
{
   THROW_BEGIN();

   void *gps_socket = scl_get_socket("gps");
   THROW_IF(gps_socket == NULL, -EIO);
   int64_t hwm = 1;
   zmq_setsockopt(gps_socket, ZMQ_SNDHWM, &hwm, sizeof(hwm));

   void *sats_socket = scl_get_socket("sats");
   THROW_IF(sats_socket == NULL, -EIO);
 
   char *platform = NULL;
   opcd_param_get("platform", &platform);
   char buffer_path[128];
   char buffer_speed[128];
   strcpy(buffer_path, platform);
   strcpy(buffer_speed, platform);
   strcat(buffer_path, ".gpsp_serial.path");
   strcat(buffer_speed, ".gpsp_serial.speed");
   char *dev_path;
   opcd_param_get(buffer_path, &dev_path);
   int dev_speed;
   opcd_param_get(buffer_speed, &dev_speed);
 
   serialport_t port;
   THROW_ON_ERR(serial_open(&port, dev_path, dev_speed, O_RDONLY));

   nmeaPARSER parser;
   nmea_parser_init(&parser);
   nmeaINFO info;
   nmea_zero_INFO(&info);

   int time_set = 0;
   int smask = 0; /* global smask collects all sentences and is never reset,
                     in contrast to info.smask */
   double lat_prev = 0.0;
   double lon_prev = 0.0;
   unsigned int count = 0;
   msgpack_sbuffer *msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);
   msgpack_packer *pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   THROW_IF(pk == NULL, -ENOMEM);

  
   while (running)
   {
      int c = serial_read_char(&port);
      if (c < 0)
         continue;
      char b = c;

      /* parse NMEA frame: */
      if (nmea_parse(&parser, &b, 1, &info) == 1)
      {
         smask |= info.smask;
         if (   (info.smask & GPGGA) /* check for new position update */
             && (smask & (GPGSA | GPRMC))) /* go sure that we collect all sentences for first output*/
         {
            /* limit sending data rate: */
            if (((count++ % 100) != 0) && convert(info.lat) == lat_prev && convert(info.lon) == lon_prev)
               continue;

            msgpack_sbuffer_clear(msgpack_buf);
            if (info.fix == 2)
               msgpack_pack_array(pk, 7); /* 2d fix */
            else if (info.fix == 3)
               msgpack_pack_array(pk, 9); /* 3d fix */
            else
               msgpack_pack_array(pk, 1); /* no fix */
            
            char time_str[TIME_STR_LEN];
            size_t len = generate_time_str(time_str, &info.utc);
            msgpack_pack_raw(pk, len);
            msgpack_pack_raw_body(pk, time_str, len); /* gps array index 0 */

            /* set system time to gps time once: */
            if (!time_set && info.fix >= 2)
            {
               char shell_date_cmd[TIME_STR_LEN + 8];
               linux_sys_set_timezone(convert(info.lat), convert(info.lon));
               sprintf(shell_date_cmd, "date -s \"%s\"", time_str);
               time_set = system(shell_date_cmd) == 0;
            }

            if (info.fix >= 2)
            {
               PACKF(convert(info.lat));  /* gps array index 1 */
               PACKF(convert(info.lon));  /* gps array index 2 */
               PACKI(info.satinfo.inuse); /* gps array index 3 */
               PACKF(info.speed);         /* gps array index 4 */
               PACKF(info.track);         /* gps array index 5 */
               PACKF(info.HDOP);          /* gps array index 6 */
            } 

            if (info.fix == 3)
            {
               PACKF(info.elv);  /* gps array index 7 */
               PACKF(info.VDOP); /* gps array index 8 */
            }

            scl_copy_send_dynamic(gps_socket, msgpack_buf->data, msgpack_buf->size);


            msgpack_sbuffer_clear(msgpack_buf);
            msgpack_pack_array(pk, info.satinfo.inview);
            FOR_N(i, info.satinfo.inview)
            {
               msgpack_pack_array(pk, 5); /* sat = [0, 1, 2, 3, 4] */
               nmeaSATELLITE *nmea_satinfo = &info.satinfo.sat[i];
               PACKI(nmea_satinfo->id);       /* sat array index 0 */
               PACKB(info.satinfo.in_use[i]); /* sat array index 1 */
               PACKI(nmea_satinfo->elv);      /* sat array index 2 */
               PACKI(nmea_satinfo->azimuth);  /* sat array index 3 */
               PACKI(nmea_satinfo->sig);      /* sat array index 4 */
            }

            scl_copy_send_dynamic(sats_socket, msgpack_buf->data, msgpack_buf->size);
         }
      }
   }

   THROW_END();
}

