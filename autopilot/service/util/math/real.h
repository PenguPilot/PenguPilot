

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
#else /* double */
   typedef double real_t;
   #define real_sqrt sqrt
   #define real_sin sin
   #define real_cos cos
#endif
#define REAL(c) ((real_t)c)


#endif /* __REAL_H__ */

