
#include <stdio.h>
#include <assert.h>
#include <util.h>


#include "fs_parser.h"
#include "coupling_matrix_parser.h"


void coupling_matrix_print(int n_motors, float mixer[FORCES_AND_MOMENTS][MAX_MOTORS])
{
   FOR_N(i, n_motors)
   {
      printf("%.2f\t%.2f\t%.2f\t%.2f\n", mixer[0][i], mixer[1][i], mixer[2][i], mixer[3][i]);   
   }
}


int coupling_matrix_parser_run(char *buffer, float mixer[FORCES_AND_MOMENTS][MAX_MOTORS])
{
   fs_parser_t parser;
   fs_parser_init(&parser);
   unsigned int n = fs_parser_count(&parser, buffer);
   int motors = n / FORCES_AND_MOMENTS;
   assert(motors < MAX_MOTORS);
   if ((n % FORCES_AND_MOMENTS) == 0)
   {
      for (int motor = 0; motor < motors; motor++)
      {
         for (int i = 0; i < FORCES_AND_MOMENTS; i++)
         {
             while (1)
             {
                fs_parser_run(&parser, *buffer++);
                if (fs_parser_ready(&parser))
                   break;
             }
             mixer[i][motor] = fs_parser_val(&parser);
         }
      }
   }
   return motors;
}

