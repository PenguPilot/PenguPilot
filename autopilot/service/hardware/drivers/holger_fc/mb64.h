/*
 * mb64.h
 *
 *  Created on: 14.06.2010
 *      Author: tobi
 */

#ifndef COCO_MB64_H
#define COCO_MB64_H


#include <stdlib.h>

#include "frame.h"


int mb64_encode(char *output, const plchar_t *input, const size_t input_len);

int mb64_decode(plchar_t *output, const size_t len, const char *input);


#endif /* COCO_MB64_H */
