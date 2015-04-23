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
  
 Calibration Interface

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__


typedef struct
{
   size_t dim; /* dimension of the calibration data */
   float *sum; /* sum vector */
   float *bias; /* bias vector */
   size_t max_samples; /* maximum number of samples */
   size_t sample; /* current sample counter */
}
calibration_t;


void cal_init(calibration_t *cal, const size_t dim, const size_t max_samples);

void cal_reset(calibration_t *cal);

int cal_sample_apply(calibration_t *cal, float *vec);


#endif /* __CALIBRATION_H__ */

