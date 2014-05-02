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
  
 Arduino PPM Parser Implementation

 Copyright (C) 2014 Jan Roemisch, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <assert.h>
#include <stdio.h>

#include "ppm_common.h"
#include "checksum.h"

static enum
{
	READ_SYNC0,
	READ_SYNC1,
	READ_CHAN_COUNT,
	READ_CHAN0,
	READ_CHAN1,
	READ_CS0,
	READ_CS1
} state = READ_SYNC0;

static uint8_t chan_count;
static uint8_t chan_index;
static uint16_t cs;

int ppm_parse_frame(uint16_t *channels, uint8_t c)
{
	int ret = 0;

	assert(channels);

	switch(state)
	{
		case READ_SYNC0:
			if(c == (uint8_t)(PPM_PREAMBLE & 0xFF))
			{
				state = READ_SYNC1;
			}
			break;
		case READ_SYNC1:
			if(c == (uint8_t)(PPM_PREAMBLE >> 8))
			{
				state = READ_CHAN_COUNT;
			}
			else
			{
				state = READ_SYNC0;
			}
			break;
		case READ_CHAN_COUNT:
			if(c > PPM_CHAN_MAX)
			{
				state = READ_SYNC0;
			}
			else
			{
				chan_count = c;
				chan_index = 0;
				state = READ_CHAN0;
			}
			break;
		case READ_CHAN0:
			channels[chan_index] = c;
			state = READ_CHAN1;
			break;
		case READ_CHAN1:
			channels[chan_index++] |= c << 8;
			if(chan_index == chan_count)
			{
				state = READ_CS0;
			}
			else
			{
				state = READ_CHAN0;
			}
			break;
		case READ_CS0:
			cs = c;
			state = READ_CS1;
			break;
		case READ_CS1:
			cs |= c << 8;
			if(cs == checksum((uint8_t *)(channels), chan_count * sizeof(uint16_t)))
			{
				ret = chan_count;
			}
			state = READ_SYNC0;
			break;
	}

	return ret;
}

