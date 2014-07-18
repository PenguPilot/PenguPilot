

#include <util.h>
#include <float.h>

#include "algo.h"


float find_minimum(float *array, size_t size)
{
   float min = FLT_MAX;
   FOR_N(i, size)
      if (array[i] < min)
         min = array[i];
   return min;
}


float find_maximum(float *array, size_t size)
{
   float max = 0.0f;
   FOR_N(i, size)
      if (array[i] > max)
         max = array[i];
   return max;
}


