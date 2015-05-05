
#ifndef __COUPLING_MATRIX_PARSER_H__
#define __COUPLING_MATRIX_PARSER_H__


#define FORCES_AND_MOMENTS (4)


#include "motors.h"


int coupling_parser_run(char *definition, float mixer[FORCES_AND_MOMENTS][MAX_MOTORS]);


#endif /* __COUPLING_MATRIX_PARSER_H__ */

