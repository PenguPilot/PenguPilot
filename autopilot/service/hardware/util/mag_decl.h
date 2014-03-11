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
  
 
 Magnetic Declination Lookup Table Interface
 
 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology
 Copyright (C) Adam M Rivera
 With direction from: Andrew Tridgell, Jason Short, Justin Beech
 Adapted from: http://www.societyofrobots.com/robotforum/index.php?topic=11855.0
 Scott Ferguson
 scottfromscott@gmail.com
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __MAG_DECL_H__
#define __MAG_DECL_H__


/* returns magnetic declination in radians
   for a given latitude/longitude in degrees */
float mag_decl_lookup(float lat, float lon);


#endif // __MAG_DECL_H__

