
#ifndef __FS_PARSER_H__
#define __FS_PARSER_H__


typedef struct
{
   int status;
   char buf[64];
   int pos;
   double val;
}
fs_parser_t;


void fs_parser_init(fs_parser_t *parser);

void fs_parser_run(fs_parser_t *parser, char c);

int fs_parser_end(fs_parser_t *parser);

int fs_parser_ready(fs_parser_t *parser);

float fs_parser_val(fs_parser_t *parser);

int fs_parser_count(fs_parser_t *parser, char *p);


#endif /* __FS_PARSER_H__ */

