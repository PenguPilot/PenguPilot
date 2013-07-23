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
  
 XY Speed Controller Implementation

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "xy_speed.h"

#include <math.h>
#include <util.h>
#include <threadsafe_types.h>
#include <opcd_interface.h>
#include <meschach/matrix.h>


typedef struct
{
   MAT *matrix; /* rotation matrix */
   VEC *in; /* rotation input vector */
   VEC *out; /* rotation output vector */
}
rot2_t;


static void rot2_init(rot2_t *rot2)
{
   rot2->matrix = m_get(2, 2);
   ASSERT_NOT_NULL(rot2->matrix);
   rot2->in = v_get(2);
   ASSERT_NOT_NULL(rot2->in);
   rot2->out = v_get(2);
   ASSERT_NOT_NULL(rot2->out);
}


static void rot2_calc(rot2_t *rot2, vec2_t *out, vec2_t *in, float angle)
{
   ASSERT_NOT_NULL(rot2->matrix);
   ASSERT_NOT_NULL(rot2->in);
   ASSERT_NOT_NULL(rot2->out);

   /* copy arguments to input vector: */
   rot2->in->ve[0] = in->x;
   rot2->in->ve[1] = in->y;

   /*
    * build rotation matrix:
    *
    * | cos(x) -sin(x) |
    * | sin(x)  cos(x) |
    */
   rot2->matrix->me[0][0] =  cosf(angle);
   rot2->matrix->me[0][1] =  sinf(angle);
   rot2->matrix->me[1][0] = -sinf(angle);
   rot2->matrix->me[1][1] =  cosf(angle);

   /* perform rotation and copy result into output: */
   vm_mlt(rot2->matrix, rot2->in, rot2->out);
   out->x = rot2->out->ve[0];
   out->y = rot2->out->ve[1];
}


static rot2_t rot2;
static tsfloat_t p;
static tsfloat_t angle_max;


void xy_speed_ctrl_init(void)
{
   ASSERT_ONCE();
   opcd_param_t params[] =
   {
      {"p", &p},
      {"angle_max", &angle_max},
      OPCD_PARAMS_END
   };
   opcd_params_apply("controllers.xy_speed.", params);
   rot2_init(&rot2);
}


void xy_speed_ctrl_run(vec2_t *control, vec2_t *speed_setpoint, vec2_t *speed, float yaw)
{
   /* calculate 2d speed error: */
   vec2_t speed_err;
   vec2_sub(&speed_err, speed_setpoint, speed);
   
   /* calculate 2d speed feedback (angle): */
   vec2_t world_thrust;
   vec2_scale(&world_thrust, &speed_err, tsfloat_get(&p));

   /* rotate global speed feedback into local control primitives: */
   rot2_calc(&rot2, control, &world_thrust, yaw);
   FOR_N(i, 2) control->vec[i] = sym_limit(control->vec[i], deg2rad(tsfloat_get(&angle_max)));
}

