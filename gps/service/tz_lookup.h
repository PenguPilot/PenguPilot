
#ifndef __TZ_LOOKUP_H__
#define __TZ_LOOKUP_H__


typedef struct
{
   float lat;
   float lon;
   char *tz;
}
tz_lookup_entry_t;


#define TZ_LOOKUP_ENTRIES (22643)


extern tz_lookup_entry_t tz_lookup[TZ_LOOKUP_ENTRIES];


#endif /* __TZ_LOOKUP_H__ */


