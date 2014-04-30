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
  
 Blackbox Publisher Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "blackbox.h"

#include <scl.h>
#include <util.h>
#include <msgpack.h>


#define PACKI(val) msgpack_pack_int(pk, val) /* pack integer */
#define PACKF(val) msgpack_pack_float(pk, val) /* pack float */
#define PACKD(val) msgpack_pack_double(pk, val) /* pack double */
#define PACKFV(ptr, n) FOR_N(i, n) PACKF(ptr[i]) /* pack float vector */


static void *blackbox_socket = NULL;
static msgpack_sbuffer *msgpack_buf = NULL;
static msgpack_packer *pk = NULL;


char *blackbox_spec[BLACKBOX_ITEMS] =
{
   "dt", /* time delta */
   "gyro_x", "gyro_y", "gyro_z", /* gyro */
   "acc_x", "acc_y", "acc_z", /* acc */
   "mag_x", "mag_y", "mag_z", /* mag */
   "lat", "lon", "alt", /* gps */
   "gps_course", "gps_speed", /* gps */
   "ultra", "baro", /* ultra / baro */
   "voltage", /* voltage */
   "current", /* current */
   "rc_pitch", "rc_roll", "rc_yaw", "rc_gas", "rc_sw_l", "rc_sw_r", /* rc */
   "sensor_status", /* sensors */
   "n_err", "e_err", "u_err", /* 3d position error */
   "n_spd_err", "e_spd_err", "u_spd_err", /* 3d speed error */
   "mag_cal_x", "mag_cal_y", "mag_cal_z" /* calibrated mag vector */
};


void blackbox_init(void)
{
   ASSERT_ONCE();
   ASSERT_NULL(blackbox_socket);
   /* get scl socket */
   blackbox_socket = scl_get_socket("blackbox");
   ASSERT_NOT_NULL(blackbox_socket);

   /* send blackbox header: */
   ASSERT_NULL(msgpack_buf);
   msgpack_buf = msgpack_sbuffer_new();
   ASSERT_NOT_NULL(msgpack_buf);
   ASSERT_NULL(pk);
   pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   ASSERT_NOT_NULL(pk);
   msgpack_pack_array(pk, ARRAY_SIZE(blackbox_spec));
   
   ASSERT_NOT_NULL(blackbox_spec);
   FOR_EACH(i, blackbox_spec)
   {
      size_t len = strlen(blackbox_spec[i]);
      msgpack_pack_raw(pk, len);
      msgpack_pack_raw_body(pk, blackbox_spec[i], len);
   }
   scl_copy_send_dynamic(blackbox_socket, msgpack_buf->data, msgpack_buf->size);
}


void blackbox_record(const float dt, /* sensor inputs ... */
               const marg_data_t *marg_data,
               const gps_data_t *gps_data,
               const float ultra,
               const float baro,
               const float voltage,
               const float current,
               const float channels[MAX_CHANNELS],
               const uint16_t sensor_status,
               const vec2_t *ne_pos_err, /* NEU position errors ... */
               const float u_pos_err,
               const vec2_t *ne_spd_err, /* NEU speed errors ... */
               const float u_spd_err,
               const vec3_t *mag_normal)
{
   msgpack_sbuffer_clear(msgpack_buf);
   msgpack_pack_array(pk, ARRAY_SIZE(blackbox_spec));
   PACKF(dt);
   PACKFV(marg_data->gyro.ve, 3);
   PACKFV(marg_data->acc.ve, 3);
   PACKFV(marg_data->mag.ve, 3);
   PACKD(gps_data->lat); PACKD(gps_data->lon); PACKF(gps_data->alt);
   PACKF(gps_data->course); PACKF(gps_data->speed);
   PACKF(ultra); PACKF(baro);
   PACKF(voltage);
   PACKF(current);
   PACKFV(channels, 6);
   PACKI(sensor_status);
   PACKFV(ne_pos_err->ve, 2);
   PACKF(u_pos_err);
   PACKFV(ne_spd_err->ve, 2);
   PACKF(u_spd_err);
   PACKFV(mag_normal->ve, 3);
   scl_copy_send_dynamic(blackbox_socket, msgpack_buf->data, msgpack_buf->size);
}

