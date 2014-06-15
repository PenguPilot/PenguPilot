/*___________________________________________________
 |  _____                       _____ _ _       _    |
 | |  __ \                     |  __ (_) |     | |   |
 | | |__) |__ _ __   __ _ _   _| |__) || | ___ | |_  |
 | |  ___/ _ \ '_ \ / _` | | | |  ___/ | |/ _ \| __| |
 | | |  |  __/ | | | (_| | |_| | |   | | | (_) | |_  |
 | |_|   \___|_| |_|\__, |\__,_|_|   |_|_|\___/ \__| |
 |                   __/ |                           |
 |  GNU/Linux based |___/  Multi-Rotor UAV Autopilot |
 |___________________________________________________|
  
 GPS Util Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <math.h>

#include "../../util/math/conv.h"
#include "gps_util.h"




static int initialized;
static double start_lon; /* start longitude in meters */
static double start_lat; /* start latitude in meters */


static void gps_to_meters(float *x, float *y, double start_lat, double start_lon, double lat, double lon)
{
   double _y = (lat - start_lat) / 8.983152841195214e-06;
   double _x = (lon - start_lon) / 8.983152841195214e-06 * cos(M_PI / 180.0f * start_lat);
   *x = _x;
   *y = _y;
}


void gps_util_update(gps_rel_data_t *out, const gps_data_t *in)
{
   if (!initialized)
   {
      /* set-up start positions */
      start_lon = in->lon;
      start_lat = in->lat;
      initialized = 1;
   }

   /* calculate deltas: */
   if (initialized)
   {
      gps_to_meters(&out->de, &out->dn, start_lat, start_lon, in->lat, in->lon);
   }

   float speed_m = in->speed / 3.6; /* km/h to m/s */
   float alpha = deg2rad(in->course);
   out->speed_n = speed_m * cos(alpha);
   out->speed_e = speed_m * sin(alpha);
}



