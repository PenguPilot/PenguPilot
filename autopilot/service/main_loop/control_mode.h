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
  
 Control Modes Interface
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


#ifndef __CONTROL_MODE_H__
#define __CONTROL_MODE_H__


#include <stdbool.h>

#include "../util/math/vec2.h"


/*************************
 * GENERIC MOTOR CONTROL *
 *************************/

bool cm_motors_enabled(void);
void cm_enable_motors(bool enabled);


/*******************
 * U CONTROL MODES *
 *******************/

void cm_u_ultra_pos_set(float pos);
void cm_u_baro_pos_set(float pos);
void cm_u_spd_set(float spd);
void cm_u_acc_set(float acc);

/* true, if in u pos hold mode */
bool cm_u_is_pos(void);

/* true, if in baro position hold mode
   false, if in ultra position hold mode */
bool cm_u_is_baro_pos(void);

/* true, if in u speed control mode */
bool cm_u_is_spd(void);

/* true, if in u acc mode without feedback loop */
bool cm_u_is_acc(void);

/* returns the setpoint */
float cm_u_setp(void);

/* returns the acc limit */
float cm_u_acc_limit(void);


/*********************
 * ATT CONTROL MODES *
 *********************/

void cm_att_set_gps_pos(vec2_t pos);
void cm_att_set_gps_spd(vec2_t spd);
void cm_att_set_angles(vec2_t angles);
void cm_att_set_rates(vec2_t rates);

/* true, if GPS navigation/position-hold */
bool cm_att_is_gps_pos(void);

/* true, if GPS speed */
bool cm_att_is_gps_spd(void);

/* true, if angle control */
bool cm_att_is_angle(void);

/* true, if rate control */
bool cm_att_is_rate(void);

/* the attitude setpoint */
vec2_t cm_att_setp(void);


/*********************
 * YAW CONTROL MODES *
 *********************/

/* set yaw control mode to position */
void cm_yaw_set_pos(float pos);

/* set yaw control mode to speed */
void cm_yaw_set_spd(float spd);

/* true, if yaw control is position
   false, if yaw control is speed */
bool cm_yaw_is_pos(void);

/* returns the current yaw setpoint */
float cm_yaw_setp(void);


#endif /* __CONTROL_MODE_H__ */

