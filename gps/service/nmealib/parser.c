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

#include <nmea/parser.h>

#include <nmea/parse.h>
#include <nmea/sentence.h>
#include <nmea/conversions.h>
#include <nmea/tok.h>

#include <stdlib.h>
#include <string.h>
#include <assert.h>


/**
 * Initialise the parser.
 * Allocates a buffer.
 *
 * @param parser a pointer to the parser
 * @return true (1) - success or false (0) - fail
 */
int nmea_parser_init(nmeaPARSER *parser) {
	assert(parser);
	memset(parser, 0, sizeof(nmeaPARSER));
	return 0;
}


static int nmea_parse_frame(frame_parser_t *parser, char c)
{
	int ret = 0;
	assert(parser);

	if (c == '$') {
		/* this resets the parser, independent of its state */
		parser->state = READ_DATA_CRC_CR; 
		parser->cksum = 0;
		parser->frame_len = 0;
	}
	else {
		switch (parser->state) {

			case READ_DATA_CRC_CR:
				if (c == '*')
					parser->state = READ_CS1;
				else if (c != '\r')
					parser->cksum ^= (int)c;

			case READ_CR:
				if (c == '\r')
					parser->state = READ_LF;
				break;

			case READ_LF:
				if (c == '\n' && (parser->checksum == parser->cksum))
					ret = 1;
				break;

			case READ_CS1:
				parser->cs1 = c;
				parser->state = READ_CS2;
				break;

			case READ_CS2:
				{
					char cs_buf[2] = {parser->cs1, c};
					parser->checksum = nmea_atoi(cs_buf, 2, 16);
					parser->state = READ_CR;
					break;
				}

			case READ_START:
			default:
				break;
		}
	}

	/* put character in frame buffer: */
	parser->frame[parser->frame_len] = c;
	parser->frame_len = (parser->frame_len + 1) % sizeof(parser->frame);
	if (ret)
		parser->frame[parser->frame_len] = '\0';
	return ret;
}


/**
 * Parse a string and store the results in the nmeaINFO structure
 *
 * @param parser a pointer to the parser
 * @param s the string
 * @param len the length of the string
 * @param info a pointer to the nmeaINFO structure
 * @return the number of packets that were parsed
 */
int nmea_parse(nmeaPARSER *parser, const char *s, int len, nmeaINFO *info) {
	int parsed = 0;
	int i = 0;
	assert(parser);
	assert(s);
	assert(info);

	for (i = 0; i < len; i++) {
		frame_parser_t *fp = &parser->frame_parser;
		if (nmea_parse_frame(fp, s[i])) {
			switch (nmea_parse_get_sentence_type(fp->frame)) {
				case GPGGA:
					if (nmea_parse_GPGGA(fp->frame, fp->frame_len, &parser->gpgga)) {
						parsed++;
						nmea_GPGGA2info(&parser->gpgga, info);
					}
					break;

				case GPGSA:
					if (nmea_parse_GPGSA(fp->frame, fp->frame_len, &parser->gpgsa)) {
						parsed++;
						nmea_GPGSA2info(&parser->gpgsa, info);
					}
					break;

				case GPGSV:
					if (nmea_parse_GPGSV(fp->frame, fp->frame_len, &parser->gpgsv)) {
						parsed++;
						nmea_GPGSV2info(&parser->gpgsv, info);
					}
					break;

				case GPRMC:
					if (nmea_parse_GPRMC(fp->frame, fp->frame_len, &parser->gprmc)) {
						parsed++;
						nmea_GPRMC2info(&parser->gprmc, info);
					}
					break;

				case GPVTG:
					if (nmea_parse_GPVTG(fp->frame, fp->frame_len, &parser->gpvtg)) {
						parsed++;
						nmea_GPVTG2info(&parser->gpvtg, info);
					}
					break;

				default:
					break;
			}
		}
	}

	return parsed;
}

