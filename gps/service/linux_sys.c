
#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <math.h>


#include "linux_sys.h"
#include "tz_lookup.h"


void linux_sys_set_timezone(float lat_dest, float lon_dest)
{
   char tzname[1024];
   float dist_max = 360.0f;
   int i;
   for (i = 0; i < TZ_LOOKUP_ENTRIES; i++) 
   {
      float lat = tz_lookup[i].lat;
      float lon = tz_lookup[i].lon;
      float dist = fabs(lon - lon_dest) + fabs(lat - lat_dest);
      if (dist < dist_max)
      {
         dist_max = dist;
         strcpy(tzname, tz_lookup[i].tz);
      }
   }
   char cmd[1024];
   sprintf(cmd, "cp /usr/share/zoneinfo/%s /etc/localtime", tzname);
   system(cmd);
}

