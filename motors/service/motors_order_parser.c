
#include <stdio.h>
#include <assert.h>
#include <util.h>
#include <is_parser.h>

#include "motors_order_parser.h"


void motors_order_print(int n_motors, int order[MAX_MOTORS])
{
   FOR_N(i, n_motors)
   {
      printf("%d\n", order[i]);   
   }
}


int motors_order_parser_run(char *buffer, int order[MAX_MOTORS])
{
   is_parser_t parser;
   is_parser_init(&parser);
   unsigned int motors = is_parser_count(&parser, buffer);
   assert(motors < MAX_MOTORS);
   FOR_N(i, motors)
   {
       while (1)
       {
          is_parser_run(&parser, *buffer++);
          if (is_parser_ready(&parser))
             break;
       }
       order[i] = is_parser_val(&parser) - 1;
   }
   return motors;
}

