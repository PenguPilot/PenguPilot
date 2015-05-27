

#include "is_parser.h"

#include <ctype.h>
#include <stdlib.h>

void is_parser_init(is_parser_t *parser)
{
   parser->status = 0;
   parser->pos = 0;
   parser->val = 0.0;
}


void is_parser_run(is_parser_t *parser, char c)
{
   parser->status = 0;
   if (isdigit(c) || c == '-')
   {
      parser->buf[parser->pos++] = c;
   }
   else if (c == '\0' || c == ',')
   {
      parser->buf[parser->pos] = '\0';
      parser->val = atoi(parser->buf);
      parser->pos = 0;
      parser->status |= 1;
   }
   if (c == '\0')
      parser->status |= 2;
}

int is_parser_end(is_parser_t *parser)
{
   return (parser->status & 2) ? 1 : 0; 
}


int is_parser_ready(is_parser_t *parser)
{
   return (parser->status & 1) ? 1 : 0; 
}

float is_parser_val(is_parser_t *parser)
{
   return parser->val; 
}

int is_parser_count(is_parser_t *parser, char *p)
{
   unsigned int n = 0;
   while (1)
   {
      is_parser_run(parser, *p++);
      if (is_parser_ready(parser))
         n++;
      if (is_parser_end(parser))
         break;
   }
   return n;
}

