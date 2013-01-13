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
  
 Madgwick AHRS Algorithm Implementation
 See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms

 Copyright (C) 2012 SOH Madgwick, X-IO Technologies
 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <math.h>
#include <stdio.h>


#include "../geometry/orientation.h"
#include "ahrs.h"


void ahrs_init(ahrs_t *ahrs, float beta_start, float beta_step, float beta_end)
{
   ahrs->beta = beta_start;
   ahrs->beta_step = beta_step;
   ahrs->beta_end = beta_end;
   ahrs->quat.q0 = 1;
   ahrs->quat.q1 = 0;
   ahrs->quat.q2 = 0;
   ahrs->quat.q3 = 0;
}


static float inv_sqrt(float x)
{
   return 1.0f / sqrtf(x);   
}


static void ahrs_normalize_3(float *v0, float *v1, float *v2)
{
   float recip_norm = inv_sqrt(*v0 * *v0 + *v1 * *v1 + *v2 * *v2);
   *v0 *= recip_norm;
   *v1 *= recip_norm;
   *v2 *= recip_norm;
}


static void ahrs_normalize_4(float *v0, float *v1, float *v2, float *v3)
{
   float recip_norm = inv_sqrt(*v0 * *v0 + *v1 * *v1 + *v2 * *v2 + *v3 * *v3);
   *v0 *= recip_norm;
   *v1 *= recip_norm;
   *v2 *= recip_norm;
   *v3 *= recip_norm;
}


static void ahrs_update_imu(ahrs_t *ahrs, float gx, float gy, float gz,
                            float ax, float ay, float az, float dt)
{
   /* get rate of change of quaternion from gyroscope: */
   float qDot0 = 0.5f * (-ahrs->quat.q1 * gx - ahrs->quat.q2 * gy - ahrs->quat.q3 * gz);
   float qDot1 = 0.5f * ( ahrs->quat.q0 * gx + ahrs->quat.q2 * gz - ahrs->quat.q3 * gy);
   float qDot2 = 0.5f * ( ahrs->quat.q0 * gy - ahrs->quat.q1 * gz + ahrs->quat.q3 * gx);
   float qDot3 = 0.5f * ( ahrs->quat.q0 * gz + ahrs->quat.q1 * gy - ahrs->quat.q2 * gx);

   if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
   {
      /* normalize accelerometer measurements: */
      ahrs_normalize_3(&ax, &ay, &az);

      /* auxiliary variables to avoid repeated arithmetic: */
      float _2q0 = 2.0f * ahrs->quat.q0;
      float _2q1 = 2.0f * ahrs->quat.q1;
      float _2q2 = 2.0f * ahrs->quat.q2;
      float _2q3 = 2.0f * ahrs->quat.q3;
      float _4q0 = 4.0f * ahrs->quat.q0;
      float _4q1 = 4.0f * ahrs->quat.q1;
      float _4q2 = 4.0f * ahrs->quat.q2;
      float _8q1 = 8.0f * ahrs->quat.q1;
      float _8q2 = 8.0f * ahrs->quat.q2;
      float q0q0 = ahrs->quat.q0 * ahrs->quat.q0;
      float q1q1 = ahrs->quat.q1 * ahrs->quat.q1;
      float q2q2 = ahrs->quat.q2 * ahrs->quat.q2;
      float q3q3 = ahrs->quat.q3 * ahrs->quat.q3;

      /* gradient decent algorithm corrective step: */
      float s0 = _4q0 * q2q2 - _2q2 * ax + _4q0 * q1q1 + _2q1 * ay;
      float s1 = _4q1 * q3q3 + _2q3 * ax + 4.0f * q0q0 * ahrs->quat.q1 + _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 - _4q1 * az;
      float s2 = 4.0f * q0q0 * ahrs->quat.q2 - _2q0 * ax + _4q2 * q3q3 + _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 - _4q2 * az;
      float s3 = 4.0f * q1q1 * ahrs->quat.q3 + _2q1 * ax + 4.0f * q2q2 * ahrs->quat.q3 + _2q2 * ay;

      /* normalise step magnitude: */
      ahrs_normalize_4(&s0, &s1, &s2, &s3);
   
      /* apply feedback step: */
      qDot0 -= ahrs->beta * s0;
      qDot1 -= ahrs->beta * s1;
      qDot2 -= ahrs->beta * s2;
      qDot3 -= ahrs->beta * s3;
   }

   /* integrate rate of change to yield quaternion: */
   ahrs->quat.q0 += qDot0 * dt;
   ahrs->quat.q1 += qDot1 * dt;
   ahrs->quat.q2 += qDot2 * dt;
   ahrs->quat.q3 += qDot3 * dt;

   /* normalize quaternion: */
   ahrs_normalize_4(&ahrs->quat.q0, &ahrs->quat.q1 , &ahrs->quat.q2, &ahrs->quat.q3);
}



int ahrs_update(ahrs_t *ahrs, marg_data_t *marg_data, float dt)
{
   int ret;
   if (ahrs->beta > ahrs->beta_end)
   {
      ahrs->beta -= ahrs->beta_step;
      if (ahrs->beta < ahrs->beta_end)
      {
         ahrs->beta = ahrs->beta_end;
         ret = 1;
      }
      else
      {
         ret = -1;   
      }
   }
   else
   {
      ret = 0;   
   }
 
   float gx = marg_data->gyro.x;
   float gy = marg_data->gyro.y;
   float gz = marg_data->gyro.z;
   float ax = marg_data->acc.x;
   float ay = marg_data->acc.y;
   float az = marg_data->acc.z;
   float mx = marg_data->mag.x;
   float my = marg_data->mag.y;
   float mz = marg_data->mag.z;

   /* use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation): */
   if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
   {
      ahrs_update_imu(ahrs, gx, gy, gz, ax, ay, az, dt);
      goto out;
   }

   /* Rate of change of quaternion from gyroscope */
   float qDot0 = 0.5f * (-ahrs->quat.q1 * gx - ahrs->quat.q2 * gy - ahrs->quat.q3 * gz);
   float qDot1 = 0.5f * ( ahrs->quat.q0 * gx + ahrs->quat.q2 * gz - ahrs->quat.q3 * gy);
   float qDot2 = 0.5f * ( ahrs->quat.q0 * gy - ahrs->quat.q1 * gz + ahrs->quat.q3 * gx);
   float qDot3 = 0.5f * ( ahrs->quat.q0 * gz + ahrs->quat.q1 * gy - ahrs->quat.q2 * gx);

   if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
   {
      /* normalise accelerometer and magnetometer measurements: */
      ahrs_normalize_3(&ax, &ay, &az);
      ahrs_normalize_3(&mx, &my, &mz);

      /* auxiliary variables to avoid repeated arithmetic: */
      float _2q0mx = 2.0f * ahrs->quat.q0 * mx;
      float _2q0my = 2.0f * ahrs->quat.q0 * my;
      float _2q0mz = 2.0f * ahrs->quat.q0 * mz;
      float _2q1mx = 2.0f * ahrs->quat.q1 * mx;
      float _2q0   = 2.0f * ahrs->quat.q0;
      float _2q1   = 2.0f * ahrs->quat.q1;
      float _2q2   = 2.0f * ahrs->quat.q2;
      float _2q3   = 2.0f * ahrs->quat.q3;
      float _2q0q2 = 2.0f * ahrs->quat.q0 * ahrs->quat.q2;
      float _2q2q3 = 2.0f * ahrs->quat.q2 * ahrs->quat.q3;
      float q0q0   =   ahrs->quat.q0 * ahrs->quat.q0;
      float q0q1   =   ahrs->quat.q0 * ahrs->quat.q1;
      float q0q2   =   ahrs->quat.q0 * ahrs->quat.q2;
      float q0q3   =   ahrs->quat.q0 * ahrs->quat.q3;
      float q1q1   =   ahrs->quat.q1 * ahrs->quat.q1;
      float q1q2   =   ahrs->quat.q1 * ahrs->quat.q2;
      float q1q3   =   ahrs->quat.q1 * ahrs->quat.q3;
      float q2q2   =   ahrs->quat.q2 * ahrs->quat.q2;
      float q2q3   =   ahrs->quat.q2 * ahrs->quat.q3;
      float q3q3   =   ahrs->quat.q3 * ahrs->quat.q3;

      /* reference direction of Earth's magnetic field: */
      float hx   = mx * q0q0 - _2q0my * ahrs->quat.q3 + _2q0mz * ahrs->quat.q2 + mx * q1q1 + _2q1 * my * ahrs->quat.q2 + _2q1 * mz * ahrs->quat.q3 - mx * q2q2 - mx * q3q3;
      float hy   = _2q0mx * ahrs->quat.q3 + my * q0q0 - _2q0mz * ahrs->quat.q1 + _2q1mx * ahrs->quat.q2 - my * q1q1 + my * q2q2 + _2q2 * mz * ahrs->quat.q3 - my * q3q3;
      float _2bx = sqrt(hx * hx + hy * hy);
      float _2bz = -_2q0mx * ahrs->quat.q2 + _2q0my * ahrs->quat.q1 + mz * q0q0 + _2q1mx * ahrs->quat.q3 - mz * q1q1 + _2q2 * my * ahrs->quat.q3 - mz * q2q2 + mz * q3q3;
      float _4bx = 2.0f * _2bx;
      float _4bz = 2.0f * _2bz;

      /* Gradient decent algorithm corrective step: */
      float s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 + ax) + _2q1 * (2.0f * q0q1 + _2q2q3 + ay) - _2bz * ahrs->quat.q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * ahrs->quat.q3 + _2bz * ahrs->quat.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * ahrs->quat.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      float s1 =  _2q3 * (2.0f * q1q3 - _2q0q2 + ax) + _2q0 * (2.0f * q0q1 + _2q2q3 + ay) - 4.0f * ahrs->quat.q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 + az) + _2bz * ahrs->quat.q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * ahrs->quat.q2 + _2bz * ahrs->quat.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * ahrs->quat.q3 - _4bz * ahrs->quat.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      float s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 + ax) + _2q3 * (2.0f * q0q1 + _2q2q3 + ay) - 4.0f * ahrs->quat.q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 + az) + (-_4bx * ahrs->quat.q2 - _2bz * ahrs->quat.q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * ahrs->quat.q1 + _2bz * ahrs->quat.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * ahrs->quat.q0 - _4bz * ahrs->quat.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      float s3 =  _2q1 * (2.0f * q1q3 - _2q0q2 + ax) + _2q2 * (2.0f * q0q1 + _2q2q3 + ay) + (-_4bx * ahrs->quat.q3 + _2bz * ahrs->quat.q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * ahrs->quat.q0 + _2bz * ahrs->quat.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * ahrs->quat.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

      /* normalise step magnitude: */
      ahrs_normalize_4(&s0, &s1, &s2, &s3);

      /* apply feedback step: */
      qDot0 -= ahrs->beta * s0;
      qDot1 -= ahrs->beta * s1;
      qDot2 -= ahrs->beta * s2;
      qDot3 -= ahrs->beta * s3;
   }

   /* integrate rate of change to yield quaternion: */
   ahrs->quat.q0 += qDot0 * dt;
   ahrs->quat.q1 += qDot1 * dt;
   ahrs->quat.q2 += qDot2 * dt;
   ahrs->quat.q3 += qDot3 * dt;

   /* normalise quaternion: */
   quat_normalize(&ahrs->quat);

out:
   return ret;
}

