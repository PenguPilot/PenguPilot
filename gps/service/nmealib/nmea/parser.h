/*
 * This file is part of nmealib.
 *
 * Copyright (c) 2008 Timur Sinitsyn
 * Copyright (c) 2011 Ferry Huberts
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __NMEA_PARSER_H__
#define __NMEA_PARSER_H__

#include <nmea/info.h>
#include <nmea/sentence.h>


#ifdef  __cplusplus
extern "C" {
#endif /* __cplusplus */


/**
 * NMEA frame parser structure
 */
typedef struct 
{
   int cksum;
	char cs1;
	int checksum;
   char frame[1024];
   int frame_len;

	enum {
		READ_START,
		READ_DATA_CRC_CR,
		READ_CR,
		READ_CS1,
		READ_CS2,
		READ_LF,
	}
	state;
}
frame_parser_t;


/**
 * The parser data.
 */
typedef struct _nmeaPARSER {
   union
   {
      nmeaGPGGA gpgga;
      nmeaGPGSA gpgsa;
      nmeaGPGSV gpgsv;
      nmeaGPRMC gprmc;
      nmeaGPVTG gpvtg;
   };
   frame_parser_t frame_parser;
} nmeaPARSER;


int nmea_parser_init(nmeaPARSER *parser);
int nmea_parse(nmeaPARSER *parser, const char *buff, int buff_sz, nmeaINFO *info);

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* __NMEA_PARSER_H__ */
