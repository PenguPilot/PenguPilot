/*
 * chr-6d_commands.h
 *
 *  Created on: 30.06.2010
 *      Author: tobi
 */


#ifndef CHR6DM_COMMANDS_H
#define CHR6DM_COMMANDS_H


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


#endif /* CHR6DM_COMMANDS_H */

