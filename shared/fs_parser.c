

#include "fs_parser.h"

#include <ctype.h>
#include <stdlib.h>

void fs_parser_init(fs_parser_t *parser)
{
   parser->status = 0;
   parser->pos = 0;
   parser->val = 0.0;
}


void fs_parser_run(fs_parser_t *parser, char c)
{
   parser->status = 0;
   if (isdigit(c) || c == '-' || c == '.')
   {
      parser->buf[parser->pos++] = c;
   }
   else if (c == '\0' || c == ',')
   {
      parser->buf[parser->pos] = '\0';
      parser->val = atof(parser->buf);
      parser->pos = 0;
      parser->status |= 1;
   }
   if (c == '\0')
      parser->status |= 2;
}

int fs_parser_end(fs_parser_t *parser)
{
   return (parser->status & 2) ? 1 : 0; 
}


int fs_parser_ready(fs_parser_t *parser)
{
   return (parser->status & 1) ? 1 : 0; 
}

float fs_parser_val(fs_parser_t *parser)
{
   return parser->val; 
}

int fs_parser_count(fs_parser_t *parser, char *p)
{
   unsigned int n = 0;
   while (1)
   {
      fs_parser_run(parser, *p++);
      if (fs_parser_ready(parser))
         n++;
      if (fs_parser_end(parser))
         break;
   }
   return n;
}

