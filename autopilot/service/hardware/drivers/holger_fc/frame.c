
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
  
 Mikrokopter Serial Protocol Frame Implementation
 
 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <string.h>

#include "crc.h"
#include "mb64.h"



#define START_CHAR '#'
#define END_CHAR   '\r'



int build_frame(frame_t dest, const out_cmd_t cmd,
                const plchar_t *payload, const size_t input_len)
{
   int pos = 0;
   /* add start-of-frame, address and command: */
   dest[pos++] = START_CHAR;
   dest[pos++] = addr_encode(GET_ADDR(cmd));
   dest[pos++] = GET_CMD_CHAR(cmd);
   /* add and encode payload: */
   pos += mb64_encode(&dest[pos], payload, input_len);
   /* add CRC and end-of-frame character: */
   pos += calc_crc_chars(&dest[pos], calc_crc(dest, pos));
   dest[pos++] = END_CHAR;
   dest[pos] = '\0';
   return pos;
}


ParserStatus parse_frame(in_cmd_t *cmd, plchar_t *payload, const frame_t frame)
{
   int in_pos = 0;
   int in_len = strlen(frame);
   if (frame[in_pos++] != START_CHAR)
   {
      return PARSER_INVALID_START;
   }
   mk_address_t addr = addr_decode(frame[in_pos++]);
   if (addr < 1 || addr > 3)
   {
      return PARSER_INVALID_ADDRESS;
   }
   in_cmd_t _cmd = MERGE_ADDR_CMD(addr, frame[in_pos++]);
   if (!in_command_exists(_cmd))
   {
      return PARSER_INVALID_COMMAND;
   }
   const char *crc_chars = &frame[in_len - 3];
   crc_t crc = calc_crc(frame, in_len - 3);
   if (!crc_ok(crc, crc_chars))
   {
      return PARSER_INVALID_CRC; /* invalid crc */
   }
   (void)mb64_decode(payload, in_len - 6, &frame[in_pos]);
   *cmd = _cmd;
   return PARSER_NO_ERROR;
}

