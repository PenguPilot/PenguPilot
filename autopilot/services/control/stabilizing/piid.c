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
  
 Stabilizing PIID Controller Implementation

 Copyright (C) 2012 Alexander Barth, Ilmenau University of Technology
 Copyright (C) 2012 Benjamin Jahn, Ilmenau University of Technology
 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <util.h>
#include <threadsafe_types.h>
#include <opcd_interface.h>

#include "piid.h"
#include "../util/adams4.h"
#include "../../filters/filter.h"


static tsfloat_t att_kp;
static tsfloat_t att_ki;
static tsfloat_t att_kii;
static tsfloat_t att_kd;
static tsfloat_t yaw_kp;
static tsfloat_t yaw_ki;
static tsfloat_t yaw_kd;
static tsfloat_t filt_c_fg;

static float Ts = 0.0f;

/* integrators: */
static adams4_t int_err1;
static adams4_t int_err2;

/* filters: */
static Filter1 filter_lp_err;
static Filter1 filter_hp_err;
static Filter2 filter_ref;

/* working memory: */
static float *xi_err = NULL;
static float *xii_err = NULL;
#define CTRL_NUM_TSTEP 7
static float ringbuf[3 * CTRL_NUM_TSTEP];
static int ringbuf_idx = 0;

/* integrator enable flag: */
static int int_enable = 0;


void piid_init(float _Ts, float ff_fg, float ff_d)
{
   ASSERT_ONCE();

   opcd_param_t params[] =
   {
      {"att_kp", &att_kp},
      {"att_ki", &att_ki},
      {"att_kii", &att_kii},
      {"att_kd", &att_kd},
      {"yaw_kp", &yaw_kp},
      {"yaw_ki", &yaw_ki},
      {"yaw_kd", &yaw_kd},
      {"filt_c_fg", &filt_c_fg},
      OPCD_PARAMS_END
   };
   opcd_params_apply("controllers.stabilizing.", params);

   Ts = _Ts;

   /* initialize multistep integrator: */
   adams4_init(&int_err1, 3);
   adams4_init(&int_err2, 3);

   /* initialize error and reference signal filters: */
   filter1_lp_init(&filter_lp_err, tsfloat_get(&filt_c_fg), Ts, 3);
   filter1_hp_init(&filter_hp_err, tsfloat_get(&filt_c_fg), Ts, 3);
   filter2_lp_init(&filter_ref, ff_fg, ff_d, Ts, 3);

   /* allocate some working memory: */
   xi_err = calloc(3, sizeof(float));
   ASSERT_NOT_NULL(xi_err);
   xii_err = calloc(3, sizeof(float));
   ASSERT_NOT_NULL(xii_err);

   /* init ring buffer: */
   memset(ringbuf, 0, sizeof(ringbuf));
   ringbuf_idx = 0;
}


void piid_int_enable(int val)
{
   int_enable = val;
}


void piid_reset(void)
{
   adams4_reset(&int_err1);
   adams4_reset(&int_err2);
}


void piid_run(float u_ctrl[4], float gyro[3], float rc[3])
{
   float error[3];
   float derror[3];
   float rc_filt[3];

   /* filter reference signals */
   filter2_run(&filter_ref, rc, rc_filt);

   FOR_N(i, 3)
   {
      error[i] = ringbuf[ringbuf_idx + i] - gyro[i];
      ringbuf[ringbuf_idx + i] = rc_filt[i];
   }

   ringbuf_idx += 3;
   if (ringbuf_idx >= 3 * CTRL_NUM_TSTEP)
   {
      ringbuf_idx = 0;
   }

   /* error high/lowpass filter: */
   filter1_run(&filter_hp_err, error, derror);
   filter1_run(&filter_lp_err, error, error);

   /* 1st error integration: */
   FOR_N(i, 3)
   {
      int_err1.f0[i] = error[i];
   }
   adams4_run(&int_err1, xi_err, Ts, int_enable);

   /* 2nd error integration: */
   FOR_N(i, 3)
   {
      int_err2.f0[i] = xi_err[i];
   }
   adams4_run(&int_err2, xii_err, Ts, int_enable);

   /* attitude feedback: */
   FOR_N(i, 2)
   {
      u_ctrl[i] +=   tsfloat_get(&att_kp)  * error[i]
                   + tsfloat_get(&att_ki)  * xi_err[i]
                   + tsfloat_get(&att_kii) * xii_err[i] 
                   + tsfloat_get(&att_kd)  * derror[i];
   }
   /* yaw feedback: */
   u_ctrl[PIID_YAW] +=   tsfloat_get(&yaw_kp) * error[PIID_YAW] 
                       + tsfloat_get(&yaw_ki) * xi_err[PIID_YAW]
                       + tsfloat_get(&yaw_kd) * derror[PIID_YAW];
}

