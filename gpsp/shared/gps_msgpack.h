
#ifndef __GPS_MSGPACK_H__
#define __GPS_MSGPACK_H__


/* satellite info elements: */
#define PRN 0
#define USE 1
#define ELV 2
#define AZI 3
#define SIG 4

/* gps data elements: */
#define TIME   0 /* always */
#define LAT    1 /* 2d fix ... */
#define LON    2
#define SATS   3
#define SPEED  4
#define COURSE 5
#define HDOP   6
#define ALT    7 /* 3d fix ... */
#define VDOP   8


inline static unsigned int gps_msgpack_fix(unsigned int array_len)
{
   if (array_len > 7)
      return 3;
   else if (array_len > 1)
      return 2;
   else
      return 0;
}


#endif /* __GPS_MSGPACK_H__ */

