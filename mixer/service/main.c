
#include <stdio.h>
#include <opcd_interface.h>

#include "coupling_matrix_parser.h"


char *matrix_def;
float mixer[FORCES_AND_MOMENTS][MAX_MOTORS];


int main(int argc, char *argv[])
{
   opcd_params_init("mixer", 0);
   /* read configuration: */
   opcd_param_t params[] =
   {
      {"quad_matrix", &matrix_def},
      OPCD_PARAMS_END
   };
   opcd_params_apply(".", params);
   int n_motors = coupling_matrix_parser_run(matrix_def, mixer);
   coupling_matrix_print(n_motors, mixer);
   return 0;   
}
