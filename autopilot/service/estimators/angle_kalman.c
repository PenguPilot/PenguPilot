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


#include "angle_kalman.h"


void angle_kalman_init(angle_kalman_t *kalman)
{
   kalman->Q_angle = 0.001;
   kalman->Q_bias = 0.00001;
   kalman->R_measure = 1.0;
   kalman->angle = 0.0;
   kalman->bias = 0.0;
   kalman->P[0][0] = 1.0;
   kalman->P[0][1] = 0.0;
   kalman->P[1][0] = 0.0;
   kalman->P[1][1] = 1.0;
   kalman->K[0] = 0.0;
   kalman->K[1] = 0.0;
   kalman->y = 0.0;
   kalman->S = 0.0;
}


float angle_kalman_run(angle_kalman_t *kalman, const float newRate, const float newAngle, const float dt)
{
   kalman->rate = newRate - kalman->bias;
   kalman->angle += dt * kalman->rate;

   // Update estimation error covariance - Project the error covariance ahead
   /* Step 2 */
   kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
   kalman->P[0][1] -= dt * kalman->P[1][1];
   kalman->P[1][0] -= dt * kalman->P[1][1];
   kalman->P[1][1] += kalman->Q_bias * dt;

   // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
   // Calculate Kalman gain - Compute the Kalman gain
   /* Step 4 */
   kalman->S = kalman->P[0][0] + kalman->R_measure;
   /* Step 5 */
   kalman->K[0] = kalman->P[0][0] / kalman->S;
   kalman->K[1] = kalman->P[1][0] / kalman->S;

   // Calculate angle and bias - Update estimate with measurement zk (newAngle)
   /* Step 3 */
   kalman->y = newAngle - kalman->angle;
   /* Step 6 */
   kalman->angle += kalman->K[0] * kalman->y;
   kalman->bias += kalman->K[1] * kalman->y;

   kalman->P[0][0] -= kalman->K[0] * kalman->P[0][0];
   kalman->P[0][1] -= kalman->K[0] * kalman->P[0][1];
   kalman->P[1][0] -= kalman->K[1] * kalman->P[0][0];
   kalman->P[1][1] -= kalman->K[1] * kalman->P[0][1];

   return kalman->angle;
}

