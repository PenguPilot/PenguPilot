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
  
 Kalman Filter based Position/Speed Estimate
   
 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology
 Copyright (C) 2013 Jan Roemisch, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <stdbool.h>
#include <math.h>
#include <util.h>
#include <simple_thread.h>
#include <opcd_interface.h>
#include <threadsafe_types.h>

#include "pos.h"
#include "../util/logger/logger.h"
#include "../util/math/mat.h"


MAT_DECL(3, 3);
MAT_DECL(2, 2);
MAT_DECL(3, 1);


/* configuration parameters: */
static tsfloat_t process_noise;
static tsfloat_t ultra_noise;
static tsfloat_t baro_noise;
static tsfloat_t xy_acc_noise;
static tsfloat_t z_acc_noise;
static tsfloat_t gps_noise;
static tsfloat_t gps_speed_noise;
static tsint_t use_gps_speed;


typedef struct
{
   /* configuration and constant matrices: */
   mat3x3_t Q; /* process noise */
   mat3x3_t R; /* measurement noise */
   mat3x3_t I; /* identity matrix */

   /* state and transition vectors/matrices: */
   vec3_t x; /* state (location and velocity) */
   vec3_t z; /* measurement (location) */
   mat3x3_t P; /* error covariance */
   mat3x3_t A; /* system matrix */
   mat3x3_t H; /* observer matrix */
   mat3x3_t K; /* kalman gain */

   /*  vectors and matrices for calculations: */
   vec3_t t0;
   vec3_t t1;
   mat3x3_t T0;
   mat3x3_t T1;

   bool use_speed;
}
kalman_t;


/* static function prototypes: */
static void kalman_init(kalman_t *kf, float q, 
                        float r_pos, float r_speed, float r_acc,
                        float pos, float speed, bool use_speed);

static void kalman_run(kalman_t *kf, float *est_pos, float *est_speed,
                       float pos, float speed, float acc, float dt);


/* kalman filters: */
static kalman_t n_kalman;
static kalman_t e_kalman;
static kalman_t baro_u_kalman;
static kalman_t ultra_u_kalman;
static float ultra_prev = 0.0f;
static float baro_prev = 0.0f;


void pos_init(void)
{
   ASSERT_ONCE();

   /* read configuration and initialize scl gates: */
   opcd_param_t params[] =
   {
      {"process_noise", &process_noise},
      {"ultra_noise", &ultra_noise},
      {"baro_noise", &baro_noise},
      {"z_acc_noise", &z_acc_noise},
      {"xy_acc_noise", &xy_acc_noise},
      {"gps_speed_noise", &gps_speed_noise},
      {"gps_noise", &gps_noise},
      {"use_gps_speed", &use_gps_speed},
      OPCD_PARAMS_END
   };
   opcd_params_apply("kalman_pos.", params);
   LOG(LL_DEBUG, "process noise: %f, ultra noise: %f, baro noise: %f, gps noise: %f",
       tsfloat_get(&process_noise),
       tsfloat_get(&ultra_noise),
       tsfloat_get(&baro_noise),
       tsfloat_get(&gps_noise));

   /* set-up kalman filters: */
   kalman_init(&e_kalman, tsfloat_get(&process_noise),
              tsfloat_get(&gps_noise), tsfloat_get(&gps_speed_noise), tsfloat_get(&xy_acc_noise),
              0, 0, tsint_get(&use_gps_speed));
   
   kalman_init(&n_kalman, tsfloat_get(&process_noise),
              tsfloat_get(&gps_noise), tsfloat_get(&gps_speed_noise), tsfloat_get(&xy_acc_noise),
              0, 0, tsint_get(&use_gps_speed));
   
   kalman_init(&baro_u_kalman, tsfloat_get(&process_noise), 1.0, tsfloat_get(&z_acc_noise),
              tsfloat_get(&baro_noise), 
              0, 0, false);
   
   kalman_init(&ultra_u_kalman, tsfloat_get(&process_noise),
              tsfloat_get(&ultra_noise), 1.0, tsfloat_get(&z_acc_noise),
              0, 0, false);
}


void pos_update(pos_t *out, pos_in_t *in)
{
   ASSERT_NOT_NULL(out);
   ASSERT_NOT_NULL(in);

   /* run kalman filters: */
   kalman_run(&n_kalman,       &out->ne_pos.n,    &out->ne_speed.n,    in->pos_n,   in->speed_n, in->acc.n, in->dt);
   kalman_run(&e_kalman,       &out->ne_pos.e,    &out->ne_speed.e,    in->pos_e,   in->speed_e, in->acc.e, in->dt);
   kalman_run(&baro_u_kalman,  &out->baro_u.pos,  &out->baro_u.speed,  in->baro_u,  0.0f, in->acc.u, in->dt);
   if (fabs(in->ultra_u - ultra_prev) > 10.0 * fabs(in->baro_u - baro_prev))
      kalman_run(&ultra_u_kalman, &out->ultra_u.pos, &out->ultra_u.speed, ultra_prev + in->baro_u - baro_prev, 0.0f, in->acc.u, in->dt);
   else
      kalman_run(&ultra_u_kalman, &out->ultra_u.pos, &out->ultra_u.speed, in->ultra_u, 0.0f, in->acc.u, in->dt);
   baro_prev = out->baro_u.pos;
   ultra_prev = out->ultra_u.pos;
}


static void kalman_init(kalman_t *kf, float q, 
                        float r_pos, float r_speed, float r_acc,
                        float pos, float speed, bool use_speed)
{
   kf->use_speed = use_speed;
   /* set up temporary vectors and matrices: */
   vec3_init(&kf->t0);
   vec3_init(&kf->t1);
   mat3x3_init(&kf->T0);
   mat3x3_init(&kf->T1);
   
   mat3x3_init(&kf->I);
   mat_ident(&kf->I);

   /* set initial state: */
   vec3_init(&kf->x);
   kf->x.ve[0] = pos;
   kf->x.ve[1] = speed;

   /* no measurement or control yet: */
   vec3_init(&kf->z);

   mat3x3_init(&kf->P);
   mat_ident(&kf->P);
   
   /* set up noise: */
   mat3x3_init(&kf->Q);
   mat_scalar_mul(&kf->Q, &kf->I, q);
   mat3x3_init(&kf->R);
   kf->R.me[0][0] = r_pos; //0.001;
   kf->R.me[1][1] = r_speed; //0.01;
   kf->R.me[2][2] = r_acc; //1000.0;
   
   mat3x3_init(&kf->K);

   /* H = | 1.0   0.0   0.0 |
          | 0.0   0.0   1.0 | */
   mat3x3_init(&kf->H);
   kf->H.me[0][0] = 1.0f;
   kf->H.me[1][1] = 1.0f * use_speed;
   kf->H.me[2][2] = 1.0f;

   /* A = | 1.0  run  run  run |
          | 0.0  1.0  run  run |
          | 0.0  0.0  1.0  run |
          | 0.0  0.0  0.0  1.0 |
      note: run values are set in kalman_run */
   mat3x3_init(&kf->A);
   kf->A.me[0][0] = 1.0f;
   kf->A.me[1][1] = 1.0f;
   kf->A.me[2][2] = 1.0f;
}


static void kalman_predict(kalman_t *kf)
{
   /* x = A * x */
   mat_vec_mul(&kf->x, &kf->A, &kf->x);

   /* P = A * P * AT + Q */
   mat_mul(&kf->T0, &kf->A, &kf->P);   /* T0 = A * P */
   mmtr_mul(&kf->T1, &kf->T0, &kf->A); /* T1 = T0 * AT */
   mat_add(&kf->P, &kf->T1, &kf->Q);   /* P = T1 * Q */
}


static void kalman_correct(kalman_t *kf, float pos, float acc, float speed)
{
   kf->z.ve[0] = pos;
   kf->z.ve[1] = speed;
   kf->z.ve[2] = acc;

   /* K = P * HT * inv(H * P * HT + R) */
   mat_mul(&kf->T0, &kf->H, &kf->P);   // T0 = H * P
   mmtr_mul(&kf->T1, &kf->T0, &kf->H); // T1 = T0 * HT
   mat_add(&kf->T0, &kf->T1, &kf->R);  // T0 = T1 + R
   mat_inv(&kf->T1, &kf->T0);          // T1 = inv(T0)
   mmtr_mul(&kf->T0, &kf->P, &kf->H);  // T0 = P * HT
   mat_mul(&kf->K, &kf->T0, &kf->T1);  // K = T0 * T1

   /* x = x + K * (z - H * x) */
   mat_vec_mul(&kf->t0, &kf->H, &kf->x);  // t0 = H * x
   vec_sub(&kf->t1, &kf->z, &kf->t0);     // t1 = z - t0
   mat_vec_mul(&kf->t0, &kf->K, &kf->t1); // t0 = K * t1
   vec_add(&kf->x, &kf->x, &kf->t0);      // x = x + t0

   /* P = (I - K * H) * P */
   mat_mul(&kf->T0, &kf->K, &kf->H);  // T0 = K * H
   mat_sub(&kf->T1, &kf->I, &kf->T0); // T1 = I - T0
   mat_mul(&kf->T0, &kf->T1, &kf->P); // T0 = T1 * P
   mat_copy(&kf->P, &kf->T0);         // P = T0
}


/*
 * executes kalman predict and correct step
 */
static void kalman_run(kalman_t *kf, float *est_pos, float *est_speed, float pos, float speed, float acc, float dt)
{
   /* A = | init    dt   dt^2 / 2.0 |
          | init   init  dt         |
          | init   init  init       | */
   kf->A.me[0][1] = dt;
   kf->A.me[0][2] = dt * dt / 2.0;
   kf->A.me[1][2] = dt;

   kalman_predict(kf);
   kalman_correct(kf, pos, acc, speed);
   *est_pos = kf->x.ve[0];
   *est_speed = kf->x.ve[1];
}

