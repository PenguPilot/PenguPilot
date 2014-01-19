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
  
 CHR-6DM Commands Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <netinet/in.h>
#include <string.h>

#include "chr_6dm_cmds.h"


typedef enum
{
   CHR6DM_SET_ACTIVE_CHANNELS,
   CHR6DM_SET_SILENT_MODE,
   CHR6DM_SET_BROADCAST_MODE,
   CHR6DM_SET_GYRO_BIAS,
   CHR6DM_SET_ACC_BIAS,
   CHR6DM_SET_ACC_REF_VECTOR,
   CHR6DM_AUTO_SET_ACC_REF,
   CHR6DM_ZERO_RATE_GYROS,
   CHR6DM_SELF_TEST,
   CHR6DM_SET_START_CAL,
   CHR6DM_SET_PROCESS_COVARIANCE,
   CHR6DM_SET_MAG_COVARIANCE,
   CHR6DM_SET_ACC_COVARIANCE,
   CHR6DM_SET_EKF_CONFIG,
   CHR6DM_SET_GYRO_ALIGNMENT,
   CHR6DM_SET_ACC_ALIGNMENT,
   CHR6DM_SET_MAG_REF_VECTOR,
   CHR6DM_AUTO_SET_MAG_REF,
   CHR6DM_SET_MAG_CAL,
   CHR6DM_SET_MAG_BIAS,
   CHR6DM_SET_GYRO_SCALE,
   CHR6DM_EKF_RESET,
   CHR6DM_RESET_TO_FACTORY,
   CHR6DM_WRITE_TO_FLASH,
   CHR6DM_GET_DATA,
   CHR6DM_GET_ACTIVE_CHANNELS,
   CHR6DM_GET_BROADCAST_MODE,
   CHR6DM_GET_ACC_BIAS,
   CHR6DM_GET_ACC_REF_VECTOR,
   CHR6DM_GET_GYRO_BIAS,
   CHR6DM_GET_GYRO_SCALE,
   CHR6DM_GET_START_CAL,
   CHR6DM_GET_EKF_CONFIG,
   CHR6DM_GET_ACC_COVARIANCE,
   CHR6DM_GET_MAG_COVARIANCE,
   CHR6DM_GET_PROCESS_COVARIANCE,
   CHR6DM_GET_STATE_COVARIANCE,
   CHR6DM_GET_GYRO_ALIGNMENT,
   CHR6DM_GET_ACC_ALIGNMENT,
   CHR6DM_GET_MAG_REF_VECTOR
}
chr6dm_rxcmd_t;

#define CHR6DM_N_RX_COMMANDS \
   (CHR6DM_GET_MAG_REF_VECTOR + 1)

static struct
{
   const char *name;
   unsigned char cmd;
}
rx_cmd_table[CHR6DM_N_RX_COMMANDS] =
{
   /* idx */
   {"SET_ACTIVE_CHANNELS",    0x80}, /*  0  */
   {"SET_SILENT_MODE",        0x81}, /*  1  */
   {"SET_BROADCAST_MODE",     0x82}, /*  2  */
   {"SET_GYRO_BIAS",          0x83}, /*  3  */
   {"SET_ACC_BIAS",           0x84}, /*  4  */
   {"SET_ACC_REF_VECTOR",     0x85}, /*  5  */
   {"AUTO_SET_ACC_REF",       0x86}, /*  6  */
   {"ZERO_RATE_GYROS",        0x87}, /*  7  */
   {"SELF_TEST",              0x88}, /*  8  */
   {"SET_START_CAL",          0x89}, /*  9  */
   {"SET_PROCESS_COVARIANCE", 0x8A}, /* 10  */
   {"SET_MAG_COVARIANCE",     0x8B}, /* 11  */
   {"SET_ACC_COVARIANCE",     0x8C}, /* 12  */
   {"SET_EKF_CONFIG",         0x8D}, /* 13  */
   {"SET_GYRO_ALIGNMENT",     0x8E}, /* 14  */
   {"SET_ACC_ALIGNMENT",      0x8F}, /* 15  */
   {"SET_MAG_REF_VECTOR",     0x90}, /* 16  */
   {"AUTO_SET_MAG_REF",       0x91}, /* 17  */
   {"SET_MAG_CAL",            0x92}, /* 18  */
   {"SET_MAG_BIAS",           0x93}, /* 19  */
   {"SET_GYRO_SCALE",         0x94}, /* 20  */
   {"EKF_RESET",              0x95}, /* 21  */
   {"RESET_TO_FACTORY",       0x96}, /* 22  */
   {"WRITE_TO_FLASH",         0x97}, /* 23  */
   {"GET_DATA",               0x01}, /* 24  */
   {"GET_ACTIVE_CHANNELS",    0x02}, /* 25  */
   {"GET_BROADCAST_MODE",     0x03}, /* 26  */
   {"GET_ACC_BIAS",           0x04}, /* 27  */
   {"GET_ACC_REF_VECTOR",     0x05}, /* 28  */
   {"GET_GYRO_BIAS",          0x06}, /* 29  */
   {"GET_GYRO_SCALE",         0x07}, /* 30  */
   {"GET_START_CAL",          0x08}, /* 31  */
   {"GET_EKF_CONFIG",         0x09}, /* 32  */
   {"GET_ACC_COVARIANCE",     0x0A}, /* 33  */
   {"GET_MAG_COVARIANCE",     0x0B}, /* 34  */
   {"GET_PROCESS_COVARIANCE", 0x0C}, /* 35  */
   {"GET_STATE_COVARIANCE",   0x0D}, /* 36  */
   {"GET_GYRO_ALIGNMENT",     0x0E}, /* 37  */
   {"GET_ACC_ALIGNMENT",      0x0F}, /* 38  */
   {"GET_MAG_REF_VECTOR",     0x10}  /* 39  */
};


const char *chr6dm_get_rx_command_name(unsigned char cmd)
{
   if (cmd >= 0x80 && cmd <= 0x97)
   {
      return rx_cmd_table[cmd - 0x80].name;
   }
   else if (cmd >= 0x01 && cmd <= 0x10)
   {
      return rx_cmd_table[(cmd - 0x01) + 24].name;
   }
   else
   {
      /* unknown command */
      return NULL;
   }
}


static float float_to_netfloat(float local_float)
{
   float f;
   void *tmp = &local_float;
   unsigned int i = htonl(*((unsigned int *)tmp));
   memcpy(&f, &i, sizeof(float));
   return f;
}



static unsigned char rxcmd_lookup(chr6dm_rxcmd_t cmd)
{
   return rx_cmd_table[cmd].cmd;
}


static unsigned int chr6dm_build_frame(unsigned char *buffer, chr6dm_rxcmd_t cmd,
                                       unsigned char len, const unsigned char *data)
{
   int pos = 0;

   /* create frame header: */
   buffer[pos++] = 's';
   buffer[pos++] = 'n';
   buffer[pos++] = 'p';
   buffer[pos++] = rxcmd_lookup(cmd);
   buffer[pos++] = len;

   /* copy data into frame: */
   memcpy(buffer + pos, data, len);
   pos += len;

   /* compute checksum: */
   unsigned short checksum = 0;
   for (int i = 0; i < pos; i++)
   {
      checksum += buffer[i];
   }

   /* append checksum: */
   buffer[pos++] = checksum >> 8;
   buffer[pos++] = checksum & 0xFF;

   return pos;
}


/*
 * send a simple command (without parameters)
 */
static int chr6dm_send_simple_command(const serialport_t *port, chr6dm_rxcmd_t cmd)
{
   unsigned char frame[256];
   unsigned int len = chr6dm_build_frame(frame, cmd, 0, NULL);
   return serial_write(port, (char *)frame, len);
}


int chr6dm_self_test(const serialport_t *port)
{
   return chr6dm_send_simple_command(port, CHR6DM_SELF_TEST);
}


int chr6dm_zero_gyros(const serialport_t *port)
{
   return chr6dm_send_simple_command(port, CHR6DM_ZERO_RATE_GYROS);
}


int chr6dm_get_acc_ref(const serialport_t *port)
{
   return chr6dm_send_simple_command(port, CHR6DM_GET_ACC_REF_VECTOR);
}


int chr6dm_auto_set_acc_ref(const serialport_t *port)
{
   return chr6dm_send_simple_command(port, CHR6DM_AUTO_SET_ACC_REF);
}


int chr6dm_auto_set_mag_ref(const serialport_t *port)
{
   return chr6dm_send_simple_command(port, CHR6DM_AUTO_SET_MAG_REF);
}


int chr6dm_set_ekf_config(const serialport_t *port, unsigned char ekf_config)
{
   unsigned char frame[256];
   unsigned int len = chr6dm_build_frame(frame, CHR6DM_SET_EKF_CONFIG, 1, &ekf_config);
   return serial_write(port, (char *)frame, len);
}



int chr6dm_set_acc_ref(const serialport_t *port, unsigned short vector[3])
{
   unsigned char frame[256];
   unsigned char data[6];
   for (int i = 0; i < 3; i++)
   {
      data[i * 2] = vector[2 - i] >> 8;
      data[i * 2 + 1] = vector[2 - i] & 0xFF;
   }
   unsigned int len = chr6dm_build_frame(frame, CHR6DM_SET_ACC_REF_VECTOR, 6, data);
   return serial_write(port, (char *)frame, len);
}


int chr6dm_set_silent_mode(const serialport_t *port)
{
   return chr6dm_send_simple_command(port, CHR6DM_SET_SILENT_MODE);
}


int chr6dm_get_process_covar(const serialport_t *port)
{
   return chr6dm_send_simple_command(port, CHR6DM_GET_PROCESS_COVARIANCE);
}


int chr6dm_get_acc_covar(const serialport_t *port)
{
   return chr6dm_send_simple_command(port, CHR6DM_GET_ACC_COVARIANCE);
}


int chr6dm_get_mag_covar(const serialport_t *port)
{
   return chr6dm_send_simple_command(port, CHR6DM_GET_MAG_COVARIANCE);
}


int chr6dm_write_to_flash(const serialport_t *port)
{
   return chr6dm_send_simple_command(port, CHR6DM_WRITE_TO_FLASH);
}


int chr6dm_reset_factory(const serialport_t *port)
{
   return chr6dm_send_simple_command(port, CHR6DM_RESET_TO_FACTORY);
}


int chr6dm_ekf_reset(const serialport_t *port)
{
   return chr6dm_send_simple_command(port, CHR6DM_EKF_RESET);
}


int chr6dm_get_mag_ref_vector(const serialport_t *port)
{
   return chr6dm_send_simple_command(port, CHR6DM_GET_MAG_REF_VECTOR);
}



int chr6dm_request_acc_biases(const serialport_t *port)
{
   return chr6dm_send_simple_command(port, CHR6DM_GET_ACC_BIAS);
}


int chr6dm_set_broadcast_mode(const serialport_t *port, unsigned char interval)
{
   unsigned char frame[256];
   unsigned int len = chr6dm_build_frame(frame, CHR6DM_SET_BROADCAST_MODE, 1, &interval);
   return serial_write(port, (char *)frame, len);
}


int chr6dm_set_active_channels(const serialport_t *port, unsigned short channels)
{
   unsigned char frame[256];
   unsigned char ch_data[2] = {channels >> 8, channels & 0xFF};
   unsigned int len = chr6dm_build_frame(frame, CHR6DM_SET_ACTIVE_CHANNELS, sizeof(ch_data), ch_data);
   return serial_write(port, (char *)frame, len);
}


int chr6dm_set_mag_ref_vector(const serialport_t *port, unsigned short vector[3])
{
   unsigned char frame[256];
   unsigned char data[6];
   for (int i = 0; i < 3; i++)
   {
      data[i * 2] = vector[2 - i] >> 8;
      data[i * 2 + 1] = vector[2 - i] & 0xFF;
   }
   unsigned int len = chr6dm_build_frame(frame, CHR6DM_SET_MAG_REF_VECTOR, 6, data);
   return serial_write(port, (char *)frame, len);
}


int chr6dm_set_acc_biases(const serialport_t *port, unsigned short biases[3])
{
   unsigned char frame[256];
   unsigned char data[6];
   for (int i = 0; i < 3; i++)
   {
      data[i * 2] = biases[i] >> 8;
      data[i * 2 + 1] = biases[i] & 0xFF;
   }
   unsigned int len = chr6dm_build_frame(frame, CHR6DM_SET_ACC_BIAS, 6, data);
   return serial_write(port, (char *)frame, len);
}


static int chr6dm_set_covar(const serialport_t *port, chr6dm_rxcmd_t cmd, float covar)
{
   unsigned char frame[256];
   unsigned char data[4];
   float net_covar = float_to_netfloat(covar);
   memcpy(data, &net_covar, sizeof(data));
   unsigned int len = chr6dm_build_frame(frame, cmd, sizeof(data), data);
   return serial_write(port, (char *)frame, len);
}

int chr6dm_set_process_covar(const serialport_t *port, float covar)
{
   return chr6dm_set_covar(port, CHR6DM_SET_PROCESS_COVARIANCE, covar);
}


int chr6dm_set_mag_covar(const serialport_t *port, float covar)
{
   return chr6dm_set_covar(port, CHR6DM_SET_MAG_COVARIANCE, covar);
}


int chr6dm_set_acc_covar(const serialport_t *port, float covar)
{
   return chr6dm_set_covar(port, CHR6DM_SET_ACC_COVARIANCE, covar);
}

