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


static void meter_offset(float *dn, float *de, double lat1, double lon1, double lat2, double lon2);


static int initialized;
static double start_lon; /* start longitude in meters */
static double start_lat; /* start latitude in meters */


void gps_util_update(gps_rel_data_t *out, gps_data_t *in)
{
   if (in->fix >= FIX_2D && !initialized)
   {
      /* set-up start positions */
      initialized = 1;
      start_lon = in->lon;
      start_lat = in->lat;
   }

   /* calculate deltas: */
   if (in->fix >= FIX_2D && initialized)
   {
      meter_offset(&out->dn, &out->de,
                    in->lat, in->lon,
                    start_lat, start_lon);
   }

   float speed_m = in->speed * 0.51444444444f; /* knots to meter/s */
   out->speed_n = speed_m * cosf(deg2rad(in->course));
   out->speed_e = speed_m * sinf(deg2rad(in->course));
}


#include <math.h>
#include <stdlib.h>

/* angle conversion multipliers */
#define GPS_PI       3.1415926535897932384626433832795029
#define RAD_2_DEG 57.2957795130823208767981548141051703
#define DEG_2_RAD 0.0174532925199432957692369076848861271


/* geodetic constants */
#define WGS84A 6378137     /* equatorial radius */
#define WGS84F 298.257223563  /* flattening */
#define WGS84B 6356752.3142   /* polar radius */

#define Deg2Rad(n)   ((n) * DEG_2_RAD)


/* distance in meters between two points specified in degrees, optionally
   with initial and final bearings. */
static double earth_distance_and_bearings(double lat1, double lon1, double lat2, double lon2, double *ib, double *fb)
{
   /*
    * this is a translation of the javascript implementation of the
    * Vincenty distance formula by Chris Veness. See
    * http://www.movable-type.co.uk/scripts/latlong-vincenty.html
    */
   double a, b, f;     // WGS-84 ellipsoid params
   double L, L_P, U1, U2, s_U1, c_U1, s_U2, c_U2;
   double uSq, A, B, d_S, lambda;
   double s_L, c_L, s_S, C;
   double c_S, S, s_A, c_SqA, c_2SM;
   int i = 100;

   a = WGS84A;
   b = WGS84B;
   f = 1 / WGS84F;
   L = Deg2Rad(lon2 - lon1);
   U1 = atan((1 - f) * tan(Deg2Rad(lat1)));
   U2 = atan((1 - f) * tan(Deg2Rad(lat2)));
   s_U1 = sin(U1);
   c_U1 = cos(U1);
   s_U2 = sin(U2);
   c_U2 = cos(U2);
   lambda = L;

   do
   {
      s_L = sin(lambda);
      c_L = cos(lambda);
      s_S = sqrt((c_U2 * s_L) * (c_U2 * s_L) +
                 (c_U1 * s_U2 - s_U1 * c_U2 * c_L) *
                 (c_U1 * s_U2 - s_U1 * c_U2 * c_L));

      if (s_S == 0)
         return 0;

      c_S = s_U1 * s_U2 + c_U1 * c_U2 * c_L;
      S = atan2(s_S, c_S);
      s_A = c_U1 * c_U2 * s_L / s_S;
      c_SqA = 1 - s_A * s_A;
      c_2SM = c_S - 2 * s_U1 * s_U2 / c_SqA;

      if (isnan(c_2SM))
         c_2SM = 0;

      C = f / 16 * c_SqA * (4 + f * (4 - 3 * c_SqA));
      L_P = lambda;
      lambda = L + (1 - C) * f * s_A *
               (S + C * s_S * (c_2SM + C * c_S * (2 * c_2SM * c_2SM - 1)));
   }
   while ((fabs(lambda - L_P) > 1.0e-12) && (--i > 0));

   if (i == 0)
      return NAN;    // formula failed to converge

   uSq = c_SqA * ((a * a) - (b * b)) / (b * b);
   A = 1 + uSq / 16384 * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
   B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));
   d_S = B * s_S * (c_2SM + B / 4 *
                    (c_S * (-1 + 2 * c_2SM * c_2SM) - B / 6 * c_2SM *
                     (-3 + 4 * s_S * s_S) * (-3 + 4 * c_2SM * c_2SM)));

   if (ib != NULL)
      *ib = atan2(c_U2 * sin(lambda), c_U1 * s_U2 - s_U1 * c_U2 * cos(lambda));
   if (fb != NULL)
      *fb = atan2(c_U1 * sin(lambda), c_U1 * s_U2 * cos(lambda) - s_U1 * c_U2);

   return (WGS84B * A * (S - d_S));
}


/* distance in meters between two points specified in degrees. */
static double earth_distance(double lat1, double lon1, double lat2, double lon2)
{
   return earth_distance_and_bearings(lat1, lon1, lat2, lon2, NULL, NULL);
}


/* return offset in meters */
static void meter_offset(float *dn, float *de, double lat1, double lon1, double lat2, double lon2)
{
   double _dx = (double)earth_distance(lat1, lon1, lat1, lon2);
   double _dy = (double)earth_distance(lat1, lon1, lat2, lon1);

   if (lat1 < lat2)
   {
      _dy *= -1.0;
   }
   if (lon1 < lon2)
   {
      _dx *= -1.0;
   }
   if (isnan(_dx))
   {
      _dx = 0.0f;
   }
   if (isnan(_dy))
   {
      _dy = 0.0f;
   }
   *de = _dx;
   *dn = _dy;
}

