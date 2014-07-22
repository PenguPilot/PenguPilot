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



static void *blackbox_socket = NULL;
static msgpack_sbuffer *msgpack_buf = NULL;
static msgpack_packer *pk = NULL;


char *blackbox_spec[BLACKBOX_ITEMS] =
{
   "dt",             /*  0 */
   "gyro_x",         /*  1 */
   "gyro_y",         /*  2 */
   "gyro_z",         /*  3 */
   "acc_x",          /*  4 */
   "acc_y",          /*  5 */
   "acc_z",          /*  6 */
   "mag_x",          /*  7 */
   "mag_y",          /*  8 */
   "mag_z",          /*  9 */
   "lat",            /* 10 */
   "lon",            /* 11 */
   "alt",            /* 12 */
   "gps_course",     /* 13 */
   "gps_speed",      /* 14 */
   "ultra",          /* 15 */
   "baro",           /* 16 */
   "voltage",        /* 17 */
   "current",        /* 18 */
   "rc_pitch",       /* 19 */
   "rc_roll",        /* 20 */
   "rc_yaw",         /* 21 */
   "rc_gas",         /* 22 */
   "rc_sw_l",        /* 23 */
   "rc_sw_r",        /* 24 */
   "sensor_status",  /* 25 */
   "n_err",          /* 26 */
   "e_err",          /* 27 */
   "u_err",          /* 28 */
   "n_spd_err",      /* 29 */
   "e_spd_err",      /* 30 */
   "u_spd_err",      /* 31 */
   "mag_cal_x",      /* 32 */
   "mag_cal_y",      /* 33 */
   "mag_cal_z",      /* 34 */
   "pitch_err",      /* 35 */
   "roll_err",       /* 36 */
   "yaw_err",        /* 37 */
   "pitch_rate_err", /* 38 */
   "roll_rate_err",  /* 39 */
   "yaw_rate_err",   /* 40 */
   "pitch",          /* 41 */
   "roll",           /* 42 */
   "yaw",            /* 43 */
   "n_pos",          /* 44 */
   "e_pos",          /* 45 */
   "n_spd",          /* 46 */
   "e_spd",          /* 47 */
   "baro_u_pos",     /* 48 */
   "baro_u_spd",     /* 49 */
   "ultra_u_pos",    /* 50 */
   "ultra_u_spd",    /* 51 */
   "f_n",            /* 52 */
   "f_e",            /* 53 */
   "f_u",            /* 54 */
   "decl",           /* 55 */ 
   "elev"            /* 56 */
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
   msgpack_pack_array(pk, BLACKBOX_ITEMS);
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
               const float channels[PP_MAX_CHANNELS],
               const uint16_t sensor_status,
               const vec2_t *ne_pos_err, /* NEU position errors ... */
               const float u_pos_err,
               const vec2_t *ne_spd_err, /* NEU speed errors ... */
               const float u_spd_err,
               const vec3_t *mag_normal,
               const vec3_t *pry_err,
               const vec3_t *pry_rate_err,
               const euler_t *euler,
               const vec2_t *ne_pos,
               const vec2_t *ne_spd,
               const float baro_u_pos,
               const float baro_u_spd,
               const float ultra_u_pos,
               const float ultra_u_spd,
               const vec3_t *f_neu,
               float decl,
               float elev)
{
   msgpack_sbuffer_clear(msgpack_buf);
   msgpack_pack_array(pk, BLACKBOX_ITEMS);
   PACKF(dt);
   PACKFV(marg_data->gyro.ve, 3);
   PACKFV(marg_data->acc.ve, 3);
   PACKFV(marg_data->mag.ve, 3);
   PACKD(gps_data->lat); PACKD(gps_data->lon); PACKF(gps_data->alt);
   PACKF(gps_data->course); PACKF(gps_data->speed);
   PACKF(ultra); PACKF(baro);
   PACKF(voltage);
   PACKF(current);
   PACKFV(channels, PP_MAX_CHANNELS);
   PACKI(sensor_status);
   PACKFV(ne_pos_err->ve, 2);
   PACKF(u_pos_err);
   PACKFV(ne_spd_err->ve, 2);
   PACKF(u_spd_err);
   PACKFV(mag_normal->ve, 3);
   PACKFV(pry_err->ve, 3);
   PACKFV(pry_rate_err->ve, 3);
   PACKF(euler->pitch);
   PACKF(euler->roll);
   PACKF(euler->yaw);
   PACKFV(ne_pos->ve, 2);
   PACKFV(ne_spd->ve, 2);
   PACKF(baro_u_pos);
   PACKF(baro_u_spd);
   PACKF(ultra_u_pos);
   PACKF(ultra_u_spd);
   PACKFV(f_neu->ve, 3);
   PACKF(decl);
   PACKF(elev);
   scl_copy_send_dynamic(blackbox_socket, msgpack_buf->data, msgpack_buf->size);
}

