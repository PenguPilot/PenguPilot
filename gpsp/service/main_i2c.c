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
 
 GPS Publisher and System Time Update Service

 Copyright (C) 2011 Tobias Simon, Ilmenau University of Technology

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
#include <serial.h>
#include <gps_data.pb-c.h>
#include <scl.h>
#include <i2c/i2c.h>

#include "linux_sys.h"
#include "i2c/navigatron_gtpa010.h"


#define TIME_STR_LEN 128


static void *gps_socket;


struct sat_info_t
{
   int32_t id;
   uint8_t in_use;
   int32_t elv;
   int32_t azimuth;
   int32_t sig;
};

struct i2c_gps_data_t
{
   uint32_t fix; // fix type (0,1,2,3)
   char time[TIME_STR_LEN]; // time stamp in ISO8601 format, UTC
   double lat; // lat in degrees: +/- signifies west/east
   double lon; // lon in degrees: +/- signifies north/south
   double alt; // altitude above sea level in meters
   float hdop; // horizontal dilution of precision
   float vdop; // vertical dilution of precision
   float speed; // speed over ground
   float course; // course in degrees
   uint32_t sats; // satellites in use
   struct sat_info_t* satinfo; // satellite info CURRENTLY UNUSED!!
};
 

/*static void generate_time_str(char str[TIME_STR_LEN], nmeaTIME *time)
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
} */


static double convert(double val)
{
   double deg = ((int)(val / 100.0));
   val = deg + (val - deg * 100.0) / 60.0;
   return val;
}


void main_i2c(void)
{
   i2c_bus_t bus;
   i2c_dev_t device;

   struct i2c_gps_data_t i2c_gps_data;
   memset(&i2c_gps_data, 0, sizeof(i2c_gps_data));

   uint8_t data_w[128];
   uint8_t data_r[128];   

   gps_socket = scl_get_socket("gps");
   if (gps_socket == NULL)
   {
      syslog(LOG_CRIT, "could not get scl gate");   
      exit(EXIT_FAILURE);
   }
   int64_t hwm = 1;
   zmq_setsockopt(gps_socket, ZMQ_SNDHWM, &hwm, sizeof(hwm));

   int time_set = 0;
   int smask = 0; /* global smask collects all sentences and is never reset,
                     in contrast to info.smask */

   if(i2c_bus_open(&bus, "/dev/i2c-1"))
   {
      syslog(LOG_CRIT, "could not open i2c device");   
      exit(EXIT_FAILURE);
   }
   i2c_dev_init(&device, &bus, I2C_GPS_ADDRESS);

   while (1)
   {
		msleep(200);
		GpsData gps_data = GPS_DATA__INIT;

		/*  GPS STATUS  */
		data_w[0] = I2C_GPS_STATUS;

		if(i2c_xfer(&device,1,data_w,1,data_r))
		{
			syslog(LOG_CRIT, "could not get gps status data");
		}else
		{
			i2c_gps_data.fix = data_r[0];
			//printf("\n\ngps status: %x\n",i2c_gps_data.fix);
		}

		if(((i2c_gps_data.fix & I2C_GPS_STATUS_2DFIX)==0x2) || ((i2c_gps_data.fix & I2C_GPS_STATUS_3DFIX)==0x4))
		{

			/*  TIME  */
			data_w[0] = I2C_GPS_TIME;
			if(i2c_xfer(&device,1,data_w,4,data_r))
			{
				syslog(LOG_CRIT, "could not get gps time");
			}
			else
			{

				//printf("gps time: %ld\n", ((((long) data_r[3])<<24) | (((long) data_r[2])<<16) | (((long) data_r[1])<<8) | (data_r[0])));
			 }

			/*  LOCATION  */
			data_w[0] = I2C_GPS_LOCATION;

			if(i2c_xfer(&device,1,data_w,8,data_r))
			{
				syslog(LOG_CRIT, "could not get gps location");
			}
			else
			{
				i2c_gps_data.lat = ((((long) data_r[3])<<24) | (((long) data_r[2])<<16) | (((long) data_r[1])<<8) | (data_r[0])) / 10000000.0;
				i2c_gps_data.lon = ((((long) data_r[7])<<24) | (((long) data_r[6])<<16) | (((long) data_r[5])<<8) | (data_r[4])) / 10000000.0;
				//printf("[long]\t gps lat: %ld\n\t gps lon: %ld\n", ((((long) data_r[3])<<24) | (((long) data_r[2])<<16) | (((long) data_r[1])<<8) | (data_r[0])), ((((long) data_r[7])<<24) | (((long) data_r[6])<<16) | (((long) data_r[5])<<8) | (data_r[4])));
				//printf("[double] gps lat: %f\n\t gps lon: %f\n", i2c_gps_data.lat, i2c_gps_data.lon);
			 }

			/*  ALTITUDE  */
			if((i2c_gps_data.fix & I2C_GPS_STATUS_3DFIX)==0x4)
			{
				data_w[0] = I2C_GPS_ALTITUDE;

				if(i2c_xfer(&device,1,data_w,2,data_r))
				{
					syslog(LOG_CRIT, "could not get gps altitude");
				}
				else
				{
					i2c_gps_data.alt = (double) (((data_r[1]) << 8) | (data_r[0]));
					//printf("alt: %f",i2c_gps_data.alt);
				}
			}
				
			/* set general data: * /
			char time_str[TIME_STR_LEN];
			generate_time_str(time_str, &info.utc);
			gps_data.fix = 0;
			gps_data.time = time_str;

			/ * set system time to gps time once: /
			if (!time_set && info.fix >= 2)
			{
			   char shell_date_cmd[TIME_STR_LEN + 8];
			   linux_sys_set_timezone(convert(info.lat), convert(info.lon));
			   sprintf(shell_date_cmd, "date -s \"%s\"", time_str);
			   time_set = system(shell_date_cmd) == 0;
			}*/

			/* set position data if a minimum of satellites is seen: */
			if(((i2c_gps_data.fix & 0x0F) == I2C_GPS_STATUS_2DFIX) | ((i2c_gps_data.fix & 0x0F) == I2C_GPS_STATUS_3DFIX))
			{
			   gps_data.fix = 2;
			   //PB_SET(gps_data, hdop, info.HDOP);
			   PB_SET(gps_data, lat, convert(i2c_gps_data.lat));
			   PB_SET(gps_data, lon, convert(i2c_gps_data.lon));
			   PB_SET(gps_data, sats, i2c_gps_data.fix & 0xF0);
			   //PB_SET(gps_data, course, info.track);
			   //PB_SET(gps_data, speed, info.speed);
			}
			  
			/* set data for 3d fix: */
			if (((i2c_gps_data.fix & 0x0F) == I2C_GPS_STATUS_3DFIX))
			{
			   gps_data.fix = 3;
			   //PB_SET(gps_data, vdop, info.VDOP);
			   PB_SET(gps_data, alt, i2c_gps_data.alt);
			}
			
			/*
			/ * add satellit info: * /
			unsigned int i;
			gps_data.n_satinfo = info.satinfo.inview;
			SatInfo **satinfo = malloc(gps_data.n_satinfo * sizeof(SatInfo *));
			for (i = 0; i < gps_data.n_satinfo; i++)
			{
			   / * fill SatInfo structure: * /
			   nmeaSATELLITE *nmea_satinfo = &info.satinfo.sat[i];
			   satinfo[i] = malloc(gps_data.n_satinfo * sizeof(SatInfo));
			   sat_info__init(satinfo[i]);
			   satinfo[i]->id = nmea_satinfo->id;
			   satinfo[i]->in_use = info.satinfo.in_use[i] ? 1 : 0;
			   satinfo[i]->elv = nmea_satinfo->elv;
			   satinfo[i]->azimuth = nmea_satinfo->azimuth;
			   satinfo[i]->sig = nmea_satinfo->sig;
			}
			gps_data.satinfo = satinfo;*/

			/* send the data: */
			SCL_PACK_AND_SEND_DYNAMIC(gps_socket, gps_data, gps_data);

			/* free allocated memory: * /
			for (i = 0; i < gps_data.n_satinfo; i++)
			{
			   free(satinfo[i]);
			}
			free(satinfo);*/
		}
	}
}

