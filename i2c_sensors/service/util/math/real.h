

#ifndef __REAL_H__
#define __REAL_H__


#include <math.h>


/* config */
#define REAL_IS_FLOAT 1

#if REAL_IS_FLOAT == 1
   typedef float real_t;
   #define real_sqrt sqrtf
   #define real_sin sinf
   #define real_cos cosf
   #define REAL(c) (c##f)
#else /* double */
   typedef double real_t;
   #define real_sqrt sqrt
   #define real_sin sin
   #define real_cos cos
   #define REAL(c) (c)
#endif


#endif /* __REAL_H__ */

