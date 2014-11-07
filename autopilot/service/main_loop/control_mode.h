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

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

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
void cm_disable_motors_persistent(void);


/*******************
 * U CONTROL MODES *
 *******************/

void cm_u_set_ultra_pos(float pos);
void cm_u_set_baro_pos(float pos);
void cm_u_set_spd(float spd);
void cm_u_set_acc(float acc);
void cm_u_a_max_set(float max);

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
float cm_u_sp(void);

/* returns the u acc limit */
float cm_u_a_max(void);


/*********************
 * ATT CONTROL MODES *
 *********************/

void cm_att_set_gps_pos(const vec2_t *pos);
void cm_att_set_gps_spd(const vec2_t *spd);
void cm_att_set_angles(const vec2_t *angles);
void cm_att_set_rates(const vec2_t *rates);

/* true, if GPS navigation/position-hold */
bool cm_att_is_gps_pos(void);

/* true, if GPS speed */
bool cm_att_is_gps_spd(void);

/* true, if angle control */
bool cm_att_is_angles(void);

/* true, if rate control */
bool cm_att_is_rates(void);

/* the attitude setpoint */
void cm_att_sp(vec2_t *out);


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
float cm_yaw_sp(void);

/* module initializer */
void cm_init(void);

#endif /* __CONTROL_MODE_H__ */

