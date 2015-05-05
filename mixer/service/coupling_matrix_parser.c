#include <stdio.h>
#include <assert.h>


#include "coupling_matrix_parser.h"


int parse_mixer(char *buffer, float mixer[FORCES_AND_MOMENTS][MAX_MOTORS])
{
   unsigned int n = 1;
   char *p;
   for (p = buffer; *p; p++)
      if (*p == ',')
         n++;
   p = buffer;
   int motors = n / FORCES_AND_MOMENTS;
   assert(motors < MAX_MOTORS);
   int motor;
   if ((n % FORCES_AND_MOMENTS) == 0)
   {
      for (motor = 0; motor < motors; motor++)
         for (int i = 0; i < FORCES_AND_MOMENTS; i++)
         {
            sscanf(p, "%f", &mixer[i][motor]);
            while (*p != ',')
            {
               if (*p == '\0')
                  goto out;
               p++;
            }
            p++;
         }
   }
out:
   return motors;
}

