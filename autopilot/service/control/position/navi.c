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
  
 Navigation Implementation

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


#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <util.h>
#include <opcd_interface.h>
#include <threadsafe_types.h>

#include "navi.h"

#include "../../util/math/vec2.h"


/* configurable parameters: */
static tsfloat_t speed_min;
static tsfloat_t speed_std;
static tsfloat_t speed_max;
static tsfloat_t sqrt_shift;
static tsfloat_t sqrt_scale;
static tsfloat_t square_shift;
static tsfloat_t pos_i;
static tsfloat_t pos_i_max;
static tsfloat_t ortho_p;


/* vectors for use in navigation algorithm: */
static vec2_t pos_err_sum;
static vec2_t dest_pos;
static vec2_t prev_dest_pos;


/* setpoints: */
static tsfloat_t travel_speed;


float desired_speed(float dist)
{
   float _square_shift = tsfloat_get(&square_shift);
   
   if (dist < _square_shift)
   {
      return 0.0f;
   }
   float _sqrt_shift = tsfloat_get(&sqrt_shift);
   float _sqrt_scale = tsfloat_get(&sqrt_scale);
   float meet_dist = 1.0f / 3.0f * (4.0f * _sqrt_shift - _square_shift);
   if (dist < meet_dist)
   {
      float square_scale = _sqrt_scale / (2.0f * pow(meet_dist - _square_shift, 3.0f / 2.0f));
      return square_scale * pow(dist - _square_shift, 2.0f);
   }
   float speed = _sqrt_scale * sqrt(dist - _sqrt_shift);
   float _speed_max = tsfloat_get(&speed_max);
   if (speed > _speed_max)
   {
      speed = _speed_max;
   }

   return speed;
}


/*
 * allocates and initializes memory for navigation control subsystem
 */
void navi_init(void)
{
   ASSERT_ONCE();
   opcd_param_t params[] =
   {
      {"sqrt_shift", &sqrt_shift},
      {"sqrt_scale", &sqrt_scale},
      {"square_shift", &square_shift},
      {"speed_min", &speed_min},
      {"speed_std", &speed_std},
      {"speed_max", &speed_max},
      {"pos_i", &pos_i},
      {"pos_i_max", &pos_i_max},
      {"ortho_p", &ortho_p},
      OPCD_PARAMS_END
   };
   opcd_params_apply("controllers.navigation.", params);
   
   vec2_init(&pos_err_sum);
   vec2_init(&dest_pos);
   vec2_init(&prev_dest_pos);

   tsfloat_init(&travel_speed, 0.0f);

   navi_reset_travel_speed();
}


void navi_reset(void)
{
   vec2_set(&pos_err_sum, 0.0f, 0.0f);
}


void navi_reset_travel_speed(void)
{
   tsfloat_set(&travel_speed, tsfloat_get(&speed_std));
}


int navi_set_travel_speed(float speed)
{
   if (speed > tsfloat_get(&speed_max) || speed < tsfloat_get(&speed_min))
   {
      return -1;
   }
   tsfloat_set(&travel_speed, speed);
   return 0;
}


/*
 * executes navigation control subsystem
 */
void navi_run(vec2_t *speed_setpoint, vec2_t *err, const vec2_t *pos_sp, const vec2_t *pos, const float dt)
{
   /* set-up input vectors: */
   if (!vec_equal(&dest_pos, &prev_dest_pos))
   {
      vec_copy(&prev_dest_pos, &dest_pos);
      vec_copy(&dest_pos, pos_sp);
   }
   vec2_sub(err, &dest_pos, pos);

   /* add correction for inter-setpoint trajectory */
   vec2_t ortho_thrust;
   vec2_init(&ortho_thrust);
   if ((prev_dest_pos.e == dest_pos.e)
       && (prev_dest_pos.n == dest_pos.n))
   {
      vec2_set(&ortho_thrust, 0.0, 0.0);
   }
   else
   {
      vec2_t setpoints_dir;
      vec2_init(&setpoints_dir);
      vec2_sub(&setpoints_dir, &dest_pos, &prev_dest_pos);
      vec2_t ortho_vec;
      vec2_init(&ortho_vec);
      vec2_ortho_right(&ortho_vec, &setpoints_dir);
      vec2_t ortho_pos_err;
      vec2_init(&ortho_pos_err);
      vec2_project(&ortho_pos_err, err, &ortho_vec);
      vec2_scale(&ortho_thrust, &ortho_pos_err, tsfloat_get(&ortho_p));
   }

   /* calculate speed setpoint vector: */
   vec2_t virt_dest_pos;
   vec2_set(&virt_dest_pos, dest_pos.x, dest_pos.y);
   vec2_add(&virt_dest_pos, &dest_pos, &pos_err_sum);
   vec2_add(&virt_dest_pos, &virt_dest_pos, &ortho_thrust);
   vec2_t virt_pos_err;
   vec2_init(&virt_pos_err);
   vec2_sub(&virt_pos_err, &virt_dest_pos, pos);
   vec2_sub(speed_setpoint, &virt_dest_pos, pos);
   float target_dist = vec2_norm(speed_setpoint);

   
   /* calculate error sum for "i-part" of controller,
      if sum is below pos_i_max: */
   if (vec2_norm(&pos_err_sum) < tsfloat_get(&pos_i_max))
   {
      float _speed_max = tsfloat_get(&speed_max);
      float i_weight = (_speed_max - desired_speed(target_dist)) / _speed_max;
      vec2_t pos_err_addend;
      vec2_init(&pos_err_addend);
      vec2_scale(&pos_err_addend, err, dt * tsfloat_get(&pos_i) * i_weight);
      vec2_add(&pos_err_sum, &pos_err_sum, &pos_err_addend);
   }

   float speed_val = desired_speed(target_dist) * tsfloat_get(&travel_speed);
   vec2_t dest_dir;
   vec2_init(&dest_dir);
   vec2_normalize(&dest_dir, &virt_pos_err);
   vec2_scale(speed_setpoint, &dest_dir, speed_val);
}

