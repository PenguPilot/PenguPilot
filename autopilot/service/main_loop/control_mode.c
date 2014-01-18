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

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "control_mode.h"


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


static enum
{
   U_ULTRA_POS,
   U_BARO_POS,
   U_SPEED,
   U_ACC
}
u_mode = U_ACC;
static float u_setp = 0.0;
static float u_acc_limit = 1.0;

void cm_u_set_ultra_pos(float pos)
{
   u_mode = U_ULTRA_POS;
   u_setp = pos;
}

void cm_u_set_baro_pos(float pos)
{
   u_mode = U_BARO_POS;
   u_setp = pos;
}

void cm_u_set_spd(float spd)
{
   u_mode = U_SPEED;
   u_setp = spd;
}

void cm_u_set_acc(float acc)
{
   u_mode = U_ACC;
   u_setp = acc;   
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

float cm_u_setp(void)
{
   return u_setp;   
}

float cm_u_acc_limit(void)
{
   return u_acc_limit;   
}



static enum
{
   ATT_GPS_POS,
   ATT_GPS_SPD,
   ATT_ANGLES,
   ATT_RATES
}
att_mode = ATT_RATES;
static vec2_t att_setp = {{0.0f, 0.0f}};

void cm_att_set_gps_pos(vec2_t pos)
{
   att_mode = ATT_GPS_POS;
   att_setp = pos;
}

void cm_att_set_gps_spd(vec2_t spd)
{
   att_mode = ATT_GPS_SPD;   
   att_setp = spd;
}

void cm_att_set_angles(vec2_t angles)
{
   att_mode = ATT_ANGLES;
   att_setp = angles;
}

void cm_att_set_rates(vec2_t rates)
{
   att_mode = ATT_RATES;
   att_setp = rates;
}

bool cm_att_is_gps_pos(void)
{
   return att_mode == ATT_GPS_POS;
}

bool cm_att_is_gps_spd(void)
{
   return att_mode == ATT_GPS_SPD;
}

bool cm_att_is_angle(void)
{
   return att_mode == ATT_GPS_SPD;   
}

bool cm_att_is_rate(void)
{
   return att_mode == ATT_RATES;   
}

vec2_t cm_att_setp(void)
{
   return att_setp;   
}


static bool is_yaw_pos = false;
static float yaw_setp = 0.0;

void cm_yaw_set_pos(float pos)
{
   is_yaw_pos = true;
   yaw_setp = pos;
}

void cm_yaw_set_spd(float spd)
{
   is_yaw_pos = false;
   yaw_setp = spd;
}

bool cm_yaw_is_pos(void)
{
   return is_yaw_pos;   
}

float cm_yaw_setp(void)
{
   return yaw_setp;  
}

