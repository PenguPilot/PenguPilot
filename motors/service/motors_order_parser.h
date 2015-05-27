
#ifndef __MOTORS_ORDER_PARSER_H__
#define __MOTORS_ORDER_PARSER_H__


#include <motors.h>


void motors_order_print(int n_motors, int order[MAX_MOTORS]);

int motors_order_parser_run(char *definition, int order[MAX_MOTORS]);


#endif /* __MOTORS_ORDER_PARSER_H__ */

