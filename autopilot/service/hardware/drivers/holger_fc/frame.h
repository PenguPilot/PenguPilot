/*
 * frame.h
 *
 *  Created on: 14.06.2010
 *      Author: tobi
 */


#ifndef FRAME_H
#define FRAME_H


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


#endif /* FRAME_H */
