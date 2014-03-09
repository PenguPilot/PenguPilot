
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
  
 Mikrokopter Serial Protocol Frame Interface
 
 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __MK_FRAME_H__
#define __MK_FRAME_H__


#include <stddef.h>

#include "commands.h"


typedef unsigned char plchar_t; /* type for non-ASCII payload */

typedef char frame_t[256];


int build_frame(frame_t dest, const out_cmd_t cmd,
                const plchar_t *payload, const size_t input_len);


typedef enum
{
   PARSER_NO_ERROR,
   PARSER_INVALID_START,
   PARSER_INVALID_ADDRESS,
   PARSER_INVALID_COMMAND,
   PARSER_INVALID_CRC
}
ParserStatus;


ParserStatus parse_frame(in_cmd_t *cmd, plchar_t *payload, const frame_t frame);


#endif /* __MK_FRAME_H__ */

