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
  
 Adams4 Integrator Interface

 Copyright (C) 2013 Alexander Barth, Ilmenau University of Technology
 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __ADAMS4_H__
#define __ADAMS4_H__


#include <stddef.h>


typedef struct 
{
   size_t dim;
   float *f0;
   float *f1;
   float *f2;
   float *f3;
}
adams4_t;


int adams4_init(adams4_t *a, const size_t dim);

void adams4_reset(adams4_t *a);

void adams4_run(adams4_t *a, float *x, float ts, int enabled);

void adams4_term(adams4_t *a);


#endif /* __ADAMS4_H__ */

