
#ifndef __SBUS_PARSER_H__
#define __SBUS_PARSER_H__


#include <stdint.h>
#include <stdbool.h>


typedef struct
{
   bool valid;
   uint8_t buffer[128];
   float channels[16];
   size_t frame_idx;
   bool zero_char_seen;
}
sbus_parser_t;


void sbus_parser_init(sbus_parser_t *parser);

bool sbus_parser_process(sbus_parser_t *parser, uint8_t c);


#endif /* __SBUS_PARSER_H__ */

