
#include <util.h>

#include "sbus_parser.h"


void sbus_parser_init(sbus_parser_t *parser)
{
   parser->valid = false;
   parser->frame_idx = 0; 
   parser->zero_char_seen = false;
}


bool sbus_parser_process(sbus_parser_t *parser, uint8_t c)
{
   if (c == 0x0F && parser->zero_char_seen)
      parser->frame_idx = 0;
   
   parser->buffer[parser->frame_idx++] = c;
	
   if (c == 0)
      parser->zero_char_seen = true;
   else
      parser->zero_char_seen = false;

   uint8_t *sbus = &parser->buffer[0];
   if (parser->frame_idx == 25)
   {
      if (sbus[23] == 0x0C)
         parser->valid = false;
      else
         parser->valid = true;
      int int_rc[16];
      int_rc[ 0] = ((sbus[ 1]      | sbus[ 2] << 8                 ) & 0x07FF);
      int_rc[ 1] = ((sbus[ 2] >> 3 | sbus[ 3] << 5                 ) & 0x07FF);
      int_rc[ 2] = ((sbus[ 3] >> 6 | sbus[ 4] << 2 | sbus[ 5] << 10) & 0x07FF);
      int_rc[ 3] = ((sbus[ 5] >> 1 | sbus[ 6] << 7                 ) & 0x07FF);
      int_rc[ 4] = ((sbus[ 6] >> 4 | sbus[ 7] << 4                 ) & 0x07FF);
      int_rc[ 5] = ((sbus[ 7] >> 7 | sbus[ 8] << 1 | sbus[ 9] <<  9) & 0x07FF);
      int_rc[ 6] = ((sbus[ 9] >> 2 | sbus[10] << 6                 ) & 0x07FF);
      int_rc[ 7] = ((sbus[10] >> 5 | sbus[11] << 3                 ) & 0x07FF);
      int_rc[ 8] = ((sbus[12]      | sbus[13] << 8                 ) & 0x07FF);
      int_rc[ 9] = ((sbus[13] >> 3 | sbus[14] << 5                 ) & 0x07FF);
      int_rc[10] = ((sbus[14] >> 6 | sbus[15] << 2 | sbus[16] << 10) & 0x07FF);
      int_rc[11] = ((sbus[16] >> 1 | sbus[17] << 7                 ) & 0x07FF);
      int_rc[12] = ((sbus[17] >> 4 | sbus[18] << 4                 ) & 0x07FF);
      int_rc[13] = ((sbus[18] >> 7 | sbus[19] << 1 | sbus[20] <<  9) & 0x07FF);
      int_rc[14] = ((sbus[20] >> 2 | sbus[21] << 6                 ) & 0x07FF);
      int_rc[15] = ((sbus[21] >> 5 | sbus[22] << 3                 ) & 0x07FF);

      float min = 172.0f;
      float max = 1811.0f;
      FOR_N(i, 16)
         parser->channels[i] = (int_rc[i] - min) / (max - min) - 0.5f;
      return true;
   }
   return false;
}

