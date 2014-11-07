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
  
 Control Modes Implementation
 used to set remote control or auto pilot inputs

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <float.h>
#include <util.h>

#include "control_mode.h"


/*
 * generic motor modes:
 */

static bool motors_enabled = false;
static bool persistent_disabled = false;

bool cm_motors_enabled(void)
{
   return motors_enabled;  
}

void cm_enable_motors(bool enabled)
{
   if (!persistent_disabled)
      motors_enabled = enabled;   
}

void cm_disable_motors_persistent(void)
{
   persistent_disabled = true;
}


/*
 * "up" control:
 */

static enum
{
   U_ULTRA_POS,
   U_BARO_POS,
   U_SPEED,
   U_ACC
}
u_mode = U_ACC;
static float u_sp = 0.0;
static float u_a_max = FLT_MAX;

void cm_u_set_ultra_pos(float pos)
{
   u_mode = U_ULTRA_POS;
   u_sp = pos;
}

void cm_u_set_baro_pos(float pos)
{
   u_mode = U_BARO_POS;
   u_sp = pos;
}

void cm_u_set_spd(float spd)
{
   u_mode = U_SPEED;
   u_sp = spd;
}

void cm_u_set_acc(float acc)
{
   u_mode = U_ACC;
   u_sp = acc;   
}

void cm_u_a_max_set(float max)
{
   u_a_max = max;   
}


bool cm_u_is_pos(void)
{
   return u_mode == U_ULTRA_POS || u_mode == U_BARO_POS;   
}

bool cm_u_is_baro_pos(void)
{
   return u_mode == U_BARO_POS;   
}

bool cm_u_is_spd(void)
{
   return u_mode == U_SPEED;
}

bool cm_u_is_acc(void)
{
   return u_mode == U_ACC;   
}

float cm_u_sp(void)
{
   return u_sp;   
}

float cm_u_a_max(void)
{
   return u_a_max;   
}


/*
 * attitude control modes:
 */

static enum
{
   ATT_GPS_POS,
   ATT_GPS_SPD,
   ATT_ANGLES,
   ATT_RATES
}
att_mode = ATT_RATES;
static vec2_t att_sp;

void cm_att_set_gps_pos(const vec2_t *pos)
{
   att_mode = ATT_GPS_POS;
   vec_copy(&att_sp, pos);
}

void cm_att_set_gps_spd(const vec2_t *spd)
{
   att_mode = ATT_GPS_SPD;
   vec_copy(&att_sp, spd);
}

void cm_att_set_angles(const vec2_t *angles)
{
   att_mode = ATT_ANGLES;
   vec_copy(&att_sp, angles);
}

void cm_att_set_rates(const vec2_t *rates)
{
   att_mode = ATT_RATES;
   vec_copy(&att_sp, rates);
}

bool cm_att_is_gps_pos(void)
{
   return att_mode == ATT_GPS_POS;
}

bool cm_att_is_gps_spd(void)
{
   return att_mode == ATT_GPS_SPD;
}

bool cm_att_is_angles(void)
{
   return att_mode == ATT_ANGLES;   
}

bool cm_att_is_rates(void)
{
   return att_mode == ATT_RATES;   
}

void cm_att_sp(vec2_t *out)
{
   vec_copy(out, &att_sp);
}


/*
 * yaw control modes:
 */

static bool is_yaw_pos = false;
static float yaw_sp = 0.0;

void cm_yaw_set_pos(float pos)
{
   is_yaw_pos = true;
   yaw_sp = pos;
}

void cm_yaw_set_spd(float spd)
{
   is_yaw_pos = false;
   yaw_sp = spd;
}

bool cm_yaw_is_pos(void)
{
   return is_yaw_pos;   
}

float cm_yaw_sp(void)
{
   return yaw_sp;  
}

void cm_init(void)
{
   ASSERT_ONCE();
   vec2_init(&att_sp);
}

