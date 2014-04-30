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
  
 Angle Kalman Filter

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology
 based on: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __ANGLE_KALMAN_H__
#define __ANGLE_KALMAN_H__


typedef struct
{
   float Q_angle; // Process noise variance for the accelerometer
   float Q_bias; // Process noise variance for the gyro bias
   float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise
   float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
   float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
   float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
   float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
   float K[2]; // Kalman gain - This is a 2x1 vector
   float y; // Angle difference
   float S; // Estimate error
}
angle_kalman_t;


void angle_kalman_init(angle_kalman_t *kalman);


float angle_kalman_run(angle_kalman_t *kalman, const float newRate, const float newAngle, const float dt);


#endif /* __ANGLE_KALMAN_H__ */

