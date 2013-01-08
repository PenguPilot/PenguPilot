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

#include "piid.h"
#include "../../filters/filter.h"


/* pitch and roll: */
#define PIID_KD  (0.02352953418f)
#define PIID_KP  (0.32876523028f)
#define PIID_KI  (2.20026754887f)
#define PIID_KII (4.37086296837f)


/* yaw: */
#define PIID_Y_KP  (0.108f)
#define PIID_Y_KD  (0.00648f)
#define PIID_Y_KI  (0.45f)
#define PIID_Y_KII (0.0f)


/* filter configuration: */
#define FILT_C_FG (10.0f)


void piid_init(piid_t *piid, float Ts)
{
   piid->Ts = Ts;

   /* initialize multistep integrator: */
   adams4_init(&piid->int_err1, 3);
   adams4_init(&piid->int_err2, 3);

   /* initialize error and reference signal filters: */
   filter1_lp_init(&piid->filter_lp_err, FILT_C_FG, Ts, 3);
   filter1_hp_init(&piid->filter_hp_err, FILT_C_FG, Ts, 3);
   filter2_lp_init(&piid->filter_ref, FILT_FF_FG,FILT_FF_D, Ts, 3);

   /* allocate some working memory: */
   piid->xi_err = calloc(3, sizeof(float));
   ASSERT_NOT_NULL(piid->xi_err);
   piid->xii_err = calloc(3, sizeof(float));
   ASSERT_NOT_NULL(piid->xii_err);

   /* init ring buffer: */
   FOR_N(i, 3 * CTRL_NUM_TSTEP)
       piid->ringbuf[i] = 0;
   piid->ringbuf_idx = 0;

   /* enable integrator by default: */
   piid->int_enable = 1;
}


void piid_reset(piid_t *piid)
{
   adams4_reset(&piid->int_err1);
   adams4_reset(&piid->int_err2);
}


void piid_run(piid_t *piid, float u_ctrl[4], float gyro[3], float rc[3])
{
   float error[3];
   float derror[3];
   float rc_filt[3];

   /* filter reference signals */
   filter2_run(&piid->filter_ref, rc, rc_filt);

   FOR_N(i, 3)
   {
      error[i] = piid->ringbuf[piid->ringbuf_idx + i] - gyro[i];
      piid->ringbuf[piid->ringbuf_idx + i] = rc_filt[i];
   }

   piid->ringbuf_idx += 3;
   if (piid->ringbuf_idx >= 3 * CTRL_NUM_TSTEP)
   {
      piid->ringbuf_idx = 0;
   }

   /* error high/lowpass filter: */
   filter1_run(&piid->filter_hp_err, error, derror);
   filter1_run(&piid->filter_lp_err, error, error);

   /* 1st error integration: */
   FOR_N(i, 3)
   {
      piid->int_err1.f0[i] = error[i];
   }
   adams4_run(&piid->int_err1, piid->xi_err, piid->Ts, piid->int_enable);

   /* 2nd error integration: */
   FOR_N(i, 3)
   {
      piid->int_err2.f0[i] = piid->xi_err[i];
   }
   adams4_run(&piid->int_err2, piid->xii_err, piid->Ts, piid->int_enable);

   /* apply feedback: */
   FOR_N(i, 2)
   {
      u_ctrl[i] += PIID_KP * error[i] + PIID_KI * piid->xi_err[i] + PIID_KII * piid->xii_err[i] + PIID_KD * derror[i];
   }
   u_ctrl[2] += PIID_Y_KP * error[2] + PIID_Y_KI * piid->xi_err[2] + PIID_Y_KII * piid->xii_err[2] + PIID_Y_KD * derror[2];
}

