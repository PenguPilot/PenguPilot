
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
  
 Mikrokopter Protocol Driver Interface
 
 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __MK_FC_H__
#define __MK_FC_H__


typedef struct
{
   float voltage;
   float signal;
   float current;
   pthread_mutex_t mutex;
} 
health_data_t;


int fc_init(void);


/*
 * reads current altitude
 */
float fc_read_alt(void);


/*
 * reads current battery voltage
 */
int fc_read_voltage(health_data_t *health);


/*
 * writes mixer input to motors (and low-level FC controllers)
 */
int fc_write_motors(float pitch, float roll, float yaw, float gas);


/*
 * spins up motors
 */
void fc_start_motors(void);


/*
 * stops motors
 */
void fc_stop_motors(void);


/*
 * reads out motor RPM
 */
void fc_read_motors_rpm(float *rpm_out);


#endif /* __MK_FC_H__ */

