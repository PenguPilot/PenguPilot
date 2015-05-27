
#ifndef __IS_PARSER_H__
#define __IS_PARSER_H__


typedef struct
{
   int status;
   char buf[64];
   int pos;
   int val;
}
is_parser_t;


void is_parser_init(is_parser_t *parser);

void is_parser_run(is_parser_t *parser, char c);

int is_parser_end(is_parser_t *parser);

int is_parser_ready(is_parser_t *parser);

float is_parser_val(is_parser_t *parser);

int is_parser_count(is_parser_t *parser, char *p);


#endif /* __IS_PARSER_H__ */

