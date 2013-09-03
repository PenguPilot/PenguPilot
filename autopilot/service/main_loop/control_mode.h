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

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

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


#include <stdint.h>

#include "../hardware/util/rc_channels.h"
#include "../util/math/vec2.h"


typedef struct
{
   struct
   {
      enum
      {
         ATT_GPS_POS,   /* GPS position*/
         ATT_GPS_SPEED, /* GPS speed */
         ATT_POS,   /* attitude angle */
         ATT_RATE   /* attitude rate */
      }
      type;
      int global;
      vec2_t setp; /* setpoint in global x/y or body frame pitch/roll direction */
   }
   att;

   struct
   {
      enum
      {
         Z_STICK, /* stick controls gas directly */
         Z_AUTO /* ultra/baro, stick limits gas */
      }
      type;
      float setp; /* setpoint from stick */
   }
   z;

   struct
   {
      enum
      {
         YAW_ANGLE,
         YAW_POI,
         YAW_STICK
      }
      type;
      float setp; /* yaw rate; to be extended  */   
   }
   yaw;

   int motors_enabled;
} 
control_mode_t;

void cm_set_att_gyro(vec2_t *setp);
void cm_set_att_acc(vec2_t *setp);
void cm_set_att_acc(vec2_t *setp);


void cm_init(void);


void cm_update(control_mode_t *cm, uint16_t sensor_status, float channels[MAX_CHANNELS]);


#endif /* __CONTROL_MODE_H__ */

