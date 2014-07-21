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

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology
 Copyright (C) 2012 SOH Madgwick, X-IO Technologies

 AHRS code fixes by Jeroen van de Mortel

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <math.h>


#include "ahrs.h"


VEC_DECL(4);


void ahrs_init(ahrs_t *ahrs, ahrs_type_t type, real_t beta_start, real_t beta_step, real_t beta_end)
{
   ahrs->type = type;
   ahrs->beta = beta_start;
   ahrs->beta_step = beta_step;
   ahrs->beta_end = beta_end;
   quat_init(&ahrs->quat);
}


static real_t inv_sqrt(real_t x)
{
   return REAL(1.0) / real_sqrt(x);   
}


static void ahrs_normalize_3(real_t *v0, real_t *v1, real_t *v2)
{
   real_t recip_norm = inv_sqrt(*v0 * *v0 + *v1 * *v1 + *v2 * *v2);
   *v0 *= recip_norm;
   *v1 *= recip_norm;
   *v2 *= recip_norm;
}


static void ahrs_update_imu(ahrs_t *ahrs, real_t gx, real_t gy, real_t gz,
                            real_t ax, real_t ay, real_t az, real_t dt)
{
   /* get rate of change of quaternion from gyroscope: */
   vec4_t qdot;
   vec4_init(&qdot);
   qdot.ve[0] = REAL(0.5) * (-ahrs->quat.q1 * gx - ahrs->quat.q2 * gy - ahrs->quat.q3 * gz);
   qdot.ve[1] = REAL(0.5) * ( ahrs->quat.q0 * gx + ahrs->quat.q2 * gz - ahrs->quat.q3 * gy);
   qdot.ve[2] = REAL(0.5) * ( ahrs->quat.q0 * gy - ahrs->quat.q1 * gz + ahrs->quat.q3 * gx);
   qdot.ve[3] = REAL(0.5) * ( ahrs->quat.q0 * gz + ahrs->quat.q1 * gy - ahrs->quat.q2 * gx);

   if (!((ax == REAL(0.0)) && (ay == REAL(0.0)) && (az == REAL(0.0))))
   {
      /* normalize accelerometer measurements: */
      ahrs_normalize_3(&ax, &ay, &az);

      /* auxiliary variables to avoid repeated arithmetic: */
      real_t _2q0 = REAL(2.0) * ahrs->quat.q0;
      real_t _2q1 = REAL(2.0) * ahrs->quat.q1;
      real_t _2q2 = REAL(2.0) * ahrs->quat.q2;
      real_t _2q3 = REAL(2.0) * ahrs->quat.q3;
      real_t _4q0 = REAL(4.0) * ahrs->quat.q0;
      real_t _4q1 = REAL(4.0) * ahrs->quat.q1;
      real_t _4q2 = REAL(4.0) * ahrs->quat.q2;
      real_t _8q1 = REAL(8.0) * ahrs->quat.q1;
      real_t _8q2 = REAL(8.0) * ahrs->quat.q2;
      real_t q0q0 = ahrs->quat.q0 * ahrs->quat.q0;
      real_t q1q1 = ahrs->quat.q1 * ahrs->quat.q1;
      real_t q2q2 = ahrs->quat.q2 * ahrs->quat.q2;
      real_t q3q3 = ahrs->quat.q3 * ahrs->quat.q3;

      /* gradient decent algorithm corrective step: */
      vec4_t s;
      vec4_init(&s);
      s.ve[0] = _4q0 * q2q2 - _2q2 * ax + _4q0 * q1q1 + _2q1 * ay;
      s.ve[1] = _4q1 * q3q3 + _2q3 * ax + REAL(4.0) * q0q0 * ahrs->quat.q1 + _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 - _4q1 * az;
      s.ve[2] = REAL(4.0) * q0q0 * ahrs->quat.q2 - _2q0 * ax + _4q2 * q3q3 + _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 - _4q2 * az;
      s.ve[3] = REAL(4.0) * q1q1 * ahrs->quat.q3 + _2q1 * ax + REAL(4.0) * q2q2 * ahrs->quat.q3 + _2q2 * ay;
      
      vec_normalize(&s);
      
      /* apply feedback step: */
      vec_scalar_mul(&s, &s, ahrs->beta);
      vec_sub(&qdot, &qdot, &s);
   }

   /* integrate rate of change to yield quaternion: */
   vec_scalar_mul(&qdot, &qdot, dt);
   vec_add(&ahrs->quat, &ahrs->quat, &qdot);
   
   /* normalise quaternion: */
   quat_normalize(&ahrs->quat);
}



int ahrs_update(ahrs_t *ahrs, const marg_data_t *marg_data, const real_t dt)
{
   int ret;
   if (ahrs->beta > ahrs->beta_end)
   {
      ahrs->beta -= ahrs->beta_step * dt;
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
 
   real_t gx = marg_data->gyro.x;
   real_t gy = marg_data->gyro.y;
   real_t gz = marg_data->gyro.z;
   real_t ax = marg_data->acc.x;
   real_t ay = marg_data->acc.y;
   real_t az = marg_data->acc.z;
   real_t mx = marg_data->mag.x;
   real_t my = marg_data->mag.y;
   real_t mz = marg_data->mag.z;

   /* use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
      or if requested upon initialization: */
   if (((mx == REAL(0.0)) && (my == REAL(0.0)) && (mz == REAL(0.0))) || ahrs->type == AHRS_ACC)
   {
      ahrs_update_imu(ahrs, gx, gy, gz, ax, ay, az, dt);
      goto out;
   }

   /* Rate of change of quaternion from gyroscope */
   vec4_t qdot;
   vec4_init(&qdot);
   qdot.ve[0] = REAL(0.5) * (-ahrs->quat.q1 * gx - ahrs->quat.q2 * gy - ahrs->quat.q3 * gz);
   qdot.ve[1] = REAL(0.5) * ( ahrs->quat.q0 * gx + ahrs->quat.q2 * gz - ahrs->quat.q3 * gy);
   qdot.ve[2] = REAL(0.5) * ( ahrs->quat.q0 * gy - ahrs->quat.q1 * gz + ahrs->quat.q3 * gx);
   qdot.ve[3] = REAL(0.5) * ( ahrs->quat.q0 * gz + ahrs->quat.q1 * gy - ahrs->quat.q2 * gx);

   if (!((ax == REAL(0.0)) && (ay == REAL(0.0)) && (az == REAL(0.0))))
   {
      /* normalise accelerometer and magnetometer measurements: */
      ahrs_normalize_3(&ax, &ay, &az);
      ahrs_normalize_3(&mx, &my, &mz);

      /* auxiliary variables to avoid repeated arithmetic: */
      real_t q0 = ahrs->quat.q0;
      real_t q1 = ahrs->quat.q1;
      real_t q2 = ahrs->quat.q2;
      real_t q3 = ahrs->quat.q3;
      real_t _2q0mx = REAL(2.0) * q0 * mx;
      real_t _2q0my = REAL(2.0) * q0 * my;
      real_t _2q0mz = REAL(2.0) * q0 * mz;
      real_t _2q1mx = REAL(2.0) * q1 * mx;
      real_t _2q0   = REAL(2.0) * q0;
      real_t _2q1   = REAL(2.0) * q1;
      real_t _2q2   = REAL(2.0) * q2;
      real_t _2q3   = REAL(2.0) * q3;
      
      real_t q0q0   = q0 * ahrs->quat.q0;
      real_t q0q1   = q0 * ahrs->quat.q1;
      real_t q0q2   = q0 * ahrs->quat.q2;
      real_t q0q3   = q0 * ahrs->quat.q3;
      real_t q1q1   = q1 * ahrs->quat.q1;
      real_t q1q2   = q1 * ahrs->quat.q2;
      real_t q1q3   = q1 * ahrs->quat.q3;
      real_t q2q2   = q2 * ahrs->quat.q2;
      real_t q2q3   = q2 * ahrs->quat.q3;
      real_t q3q3   = q3 * ahrs->quat.q3;

      /* reference direction of Earth's magnetic field: */
      real_t hx   = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
      real_t hy   = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
      real_t _2bx = sqrt(hx * hx + hy * hy);
      real_t _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
      real_t _4bx = REAL(2.0) * _2bx;
      real_t _4bz = REAL(2.0) * _2bz;
      real_t _8bx = REAL(2.0) * _2bx;
      real_t _8bz = REAL(2.0) * _2bz;


      /* Gradient decent algorithm corrective step: */
      vec4_t s;
      vec4_init(&s);
      s.ve[0] = -_2q2*(2*(q1q3 - q0q2) - ax)    +   _2q1*(2*(q0q1 + q2q3) - ay)   +  -_4bz*q2*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)   +   (-_4bx*q3+_4bz*q1)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)    +   _4bx*q2*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
      s.ve[1] = _2q3*(2*(q1q3 - q0q2) - ax) +   _2q0*(2*(q0q1 + q2q3) - ay) +   -4*q1*(2*(0.5 - q1q1 - q2q2) - az)    +   _4bz*q3*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)   + (_4bx*q2+_4bz*q0)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)   +   (_4bx*q3-_8bz*q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);             
      s.ve[2] = -_2q0*(2*(q1q3 - q0q2) - ax)    +     _2q3*(2*(q0q1 + q2q3) - ay)   +   (-4*q2)*(2*(0.5 - q1q1 - q2q2) - az) +   (-_8bx*q2-_4bz*q0)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(_4bx*q1+_4bz*q3)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*q0-_8bz*q2)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
      s.ve[3] = _2q1*(2*(q1q3 - q0q2) - ax) +   _2q2*(2*(q0q1 + q2q3) - ay)+(-_8bx*q3+_4bz*q1)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(-_4bx*q0+_4bz*q2)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);     
      
      vec_normalize(&s);
      
      /* apply feedback step: */
      vec_scalar_mul(&s, &s, ahrs->beta);
      vec_sub(&qdot, &qdot, &s);
   }

   /* integrate rate of change to yield quaternion: */
   vec_scalar_mul(&qdot, &qdot, dt);
   
   vec_add(&ahrs->quat, &ahrs->quat, &qdot);
   
   /* normalise quaternion: */
   quat_normalize(&ahrs->quat);

out:
   return ret;
}

