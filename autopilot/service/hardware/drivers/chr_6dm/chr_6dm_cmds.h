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
  
 CHR-6DM Commands Interface

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __CHR6DM_COMMANDS_H__
#define __CHR6DM_COMMANDS_H__


#include "../../../../../shared/serial.h"


const char *chr6dm_get_rx_command_name(unsigned char cmd);

int chr6dm_get_process_covar(const serialport_t *port);

int chr6dm_get_acc_covar(const serialport_t *port);

int chr6dm_get_mag_covar(const serialport_t *port);

int chr6dm_self_test(const serialport_t *port);

int chr6dm_request_acc_biases(const serialport_t *port);

int chr6dm_get_acc_ref(const serialport_t *port);

int chr6dm_set_acc_ref(const serialport_t *port, unsigned short vector[3]);

int chr6dm_zero_gyros(const serialport_t *port);

int chr6dm_set_mag_ref_vector(const serialport_t *port, unsigned short vector[3]);

int chr6dm_get_mag_ref_vector(const serialport_t *port);

int chr6dm_auto_set_mag_ref(const serialport_t *port);

int chr6dm_set_fir_corners(const serialport_t *port, const unsigned char corners[8]);

int chr6dm_set_fir_taps(const serialport_t *port, const unsigned char taps[6]);

int chr6dm_set_silent_mode(const serialport_t *port);

int chr6dm_write_to_flash(const serialport_t *port);

typedef enum
{
   TYPE_GET_DATA = 1,
   TYPE_GET_GYRO_BIAS = 2,
   TYPE_GET_ACCEL_BIAS = 3,
   TYPE_GET_FIR_CONFIG = 4,
   TYPE_GET_FIR_TAP_CONFIG = 5,
   TYPE_GET_ACTIVE_CHANNELS = 6,
   TYPE_GET_TRANSMIT_MODE = 7,
   TYPE_GET_PROCESS_WEIGHT = 8
}
chr6dm_param_t;


int chr6dm_set_broadcast_mode(const serialport_t *port, unsigned char interval);

int chr6dm_set_active_channels(const serialport_t *port, unsigned short channels);

int chr6dm_set_acc_biases(const serialport_t *port, unsigned short biases[3]);

int chr6dm_set_process_covar(const serialport_t *port, float covar);

int chr6dm_set_mag_covar(const serialport_t *port, float covar);

int chr6dm_set_acc_covar(const serialport_t *port, float covar);

int chr6dm_ekf_reset(const serialport_t *port);

int chr6dm_reset_factory(const serialport_t *port);

#define CHR6DM_EKF_CONFIG_ACC (1)
#define CHR6DM_EKF_CONFIG_MAG (2)

int chr6dm_set_ekf_config(const serialport_t *port, unsigned char ekf_config);


#endif /* __CHR6DM_COMMANDS_H__ */

