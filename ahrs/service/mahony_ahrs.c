
/*
 * Madgwick's implementation of Mayhony's AHRS algorithm.
 * See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 * (C) SOH Madgwick, 2011
 * (C) Tobias Simon, 2012
 */


#include "util.h"
#include "mahony_ahrs.h"
#include <math.h>

float inv_sqrt(float x)
{
   return 1.0f / sqrt(x);   
}

void mahony_ahrs_init(mahony_ahrs_t *ahrs, float Kp, float Ki)
{
   ahrs->twoKp = Kp * 2.0f;
   ahrs->twoKi = Ki * 2.0f;
   ahrs->integralFBx = 0.0f;
   ahrs->integralFBy = 0.0f;
   ahrs->integralFBz = 0.0f;
   ahrs->quat.q0 = 1.0f;
   ahrs->quat.q1 = 0.0f;
   ahrs->quat.q2 = 0.0f;
   ahrs->quat.q3 = 0.0f;
}


static void mahony_ahrs_update_imu(mahony_ahrs_t *ahrs, 
                                   float gx, float gy, float gz,
                                   float ax, float ay, float az, float dt)
{
   float recipNorm;
   float halfvx, halfvy, halfvz;
   float halfex, halfey, halfez;
   float qa, qb, qc;

   /* compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation): */
   if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
   {
      /* normalise accelerometer measurement: */
      recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;        

      /* estimated direction of gravity and vector perpendicular to magnetic flux: */
      halfvx = ahrs->quat.q1 * ahrs->quat.q3 - ahrs->quat.q0 * ahrs->quat.q2;
      halfvy = ahrs->quat.q0 * ahrs->quat.q1 + ahrs->quat.q2 * ahrs->quat.q3;
      halfvz = ahrs->quat.q0 * ahrs->quat.q0 - 0.5f + ahrs->quat.q3 * ahrs->quat.q3;

      /* error is sum of cross product between estimated and measured direction of gravity: */
      halfex = (ay * halfvz - az * halfvy);
      halfey = (az * halfvx - ax * halfvz);
      halfez = (ax * halfvy - ay * halfvx);

      /* compute and apply integral feedback if enabled */
      if (ahrs->twoKi > 0.0f)
      {
         ahrs->integralFBx += ahrs->twoKi * halfex * dt; /* integral error scaled by Ki */
         ahrs->integralFBy += ahrs->twoKi * halfey * dt;
         ahrs->integralFBz += ahrs->twoKi * halfez * dt;
         gx += ahrs->integralFBx; /* apply integral feedback */
         gy += ahrs->integralFBy;
         gz += ahrs->integralFBz;
      }
      else 
      {
         ahrs->integralFBx = 0.0f; /* prevent integral windup */
         ahrs->integralFBy = 0.0f;
         ahrs->integralFBz = 0.0f;
      }

      /* apply proportional feedback: */
      gx += ahrs->twoKp * halfex;
      gy += ahrs->twoKp * halfey;
      gz += ahrs->twoKp * halfez;
   }

   /* integrate rate of change of quaternion: */
   gx *= 0.5f * dt; /* pre-multiply common factors */
   gy *= 0.5f * dt;
   gz *= 0.5f * dt;
   qa = ahrs->quat.q0;
   qb = ahrs->quat.q1;
   qc = ahrs->quat.q2;
   ahrs->quat.q0 += (-qb * gx - qc * gy - ahrs->quat.q3 * gz);
   ahrs->quat.q1 += (qa * gx + qc * gz - ahrs->quat.q3 * gy);
   ahrs->quat.q2 += (qa * gy - qb * gz + ahrs->quat.q3 * gx);
   ahrs->quat.q3 += (qa * gz + qb * gy - qc * gx); 

   /* normalise quaternion: */
   recipNorm = inv_sqrt(ahrs->quat.q0 * ahrs->quat.q0 + ahrs->quat.q1 * ahrs->quat.q1 + ahrs->quat.q2 * ahrs->quat.q2 + ahrs->quat.q3 * ahrs->quat.q3);
   ahrs->quat.q0 *= recipNorm;
   ahrs->quat.q1 *= recipNorm;
   ahrs->quat.q2 *= recipNorm;
   ahrs->quat.q3 *= recipNorm;
}



void mahony_ahrs_update(mahony_ahrs_t *ahrs,
                        float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz, float dt)
{
   float recipNorm;
   float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
   float hx, hy, bx, bz;
   float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
   float halfex, halfey, halfez;
   float qa, qb, qc;

   /* use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation): */
   if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
   {
      mahony_ahrs_update_imu(ahrs, gx, gy, gz, ax, ay, az, dt);
      return;
   }

   /* compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation): */
   if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
   {
      /* normalise accelerometer measurement: */
      recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;     

      /* normalise magnetometer measurement: */
      recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
      mx *= recipNorm;
      my *= recipNorm;
      mz *= recipNorm;   

      /* Auxiliary variables to avoid repeated arithmetic: */
      q0q0 = ahrs->quat.q0 * ahrs->quat.q0;
      q0q1 = ahrs->quat.q0 * ahrs->quat.q1;
      q0q2 = ahrs->quat.q0 * ahrs->quat.q2;
      q0q3 = ahrs->quat.q0 * ahrs->quat.q3;
      q1q1 = ahrs->quat.q1 * ahrs->quat.q1;
      q1q2 = ahrs->quat.q1 * ahrs->quat.q2;
      q1q3 = ahrs->quat.q1 * ahrs->quat.q3;
      q2q2 = ahrs->quat.q2 * ahrs->quat.q2;
      q2q3 = ahrs->quat.q2 * ahrs->quat.q3;
      q3q3 = ahrs->quat.q3 * ahrs->quat.q3;   

      /* reference direction of Earth's magnetic field: */
      hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
      hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
      bx = sqrt(hx * hx + hy * hy);
      bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

      /* estimated direction of gravity and magnetic field: */
      halfvx = q1q3 - q0q2;
      halfvy = q0q1 + q2q3;
      halfvz = q0q0 - 0.5f + q3q3;
      halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
      halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
      halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  

      /* error is sum of cross product between estimated direction and measured direction of field vectors: */
      halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
      halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
      halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

      /* compute and apply integral feedback if enabled: */
      if (ahrs->twoKi > 0.0f)
      {
         ahrs->integralFBx += ahrs->twoKi * halfex * dt; /* integral error scaled by Ki */
         ahrs->integralFBy += ahrs->twoKi * halfey * dt;
         ahrs->integralFBz += ahrs->twoKi * halfez * dt;
         gx += ahrs->integralFBx; /* apply integral feedback */
         gy += ahrs->integralFBy;
         gz += ahrs->integralFBz;
      }
      else 
      {
         ahrs->integralFBx = 0.0f; /* prevent integral windup */
         ahrs->integralFBy = 0.0f;
         ahrs->integralFBz = 0.0f;
      }

      /* apply proportional feedback: */
      gx += ahrs->twoKp * halfex;
      gy += ahrs->twoKp * halfey;
      gz += ahrs->twoKp * halfez;
   }

   /* integrate rate of change of quaternion: */
   gx *= 0.5f * dt; /* pre-multiply common factors */
   gy *= 0.5f * dt;
   gz *= 0.5f * dt;
   qa = ahrs->quat.q0;
   qb = ahrs->quat.q1;
   qc = ahrs->quat.q2;
   ahrs->quat.q0 += (-qb * gx - qc * gy - ahrs->quat.q3 * gz);
   ahrs->quat.q1 += (qa * gx + qc * gz - ahrs->quat.q3 * gy);
   ahrs->quat.q2 += (qa * gy - qb * gz + ahrs->quat.q3 * gx);
   ahrs->quat.q3 += (qa * gz + qb * gy - qc * gx); 

   /* normalise quaternion: */
   recipNorm = inv_sqrt(ahrs->quat.q0 * ahrs->quat.q0 + ahrs->quat.q1 * ahrs->quat.q1 + ahrs->quat.q2 * ahrs->quat.q2 + ahrs->quat.q3 * ahrs->quat.q3);
   ahrs->quat.q0 *= recipNorm;
   ahrs->quat.q1 *= recipNorm;
   ahrs->quat.q2 *= recipNorm;
   ahrs->quat.q3 *= recipNorm;
}


