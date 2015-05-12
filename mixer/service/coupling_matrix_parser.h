
#ifndef __COUPLING_MATRIX_PARSER_H__
#define __COUPLING_MATRIX_PARSER_H__



#include <motors.h>
#include "coupling.h"

void coupling_matrix_print(int n_motors, float mixer[FORCES_AND_MOMENTS][MAX_MOTORS]);

int coupling_matrix_parser_run(char *definition, float mixer[FORCES_AND_MOMENTS][MAX_MOTORS]);


#endif /* __COUPLING_MATRIX_PARSER_H__ */

