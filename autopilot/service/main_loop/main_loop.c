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
  
 Main Loop Implementation

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <syslog.h>
#include <unistd.h>

#include <opcd_interface.h>
#include <util.h>
#include <sclhelper.h>
#include <msgpack.h>
#include <periodic_thread.h>

#include "mon.h"
#include "control_mode.h"
#include "main_loop.h"
#include "main_util.h"
#include "../util/time/interval.h"
#include "../interface/interface.h"
#include "../util/math/conv.h"
#include "../util/logger/logger.h"
#include "../estimators/cal_ahrs.h"
#include "../estimators/pos.h"
#include "../platform/ac.h"
#include "../platform/platform.h"
#include "../platform/arcade_quad.h"
#include "../hardware/util/acc_mag_cal.h"
#include "../hardware/util/calibration.h"
#include "../hardware/util/gps_util.h"
#include "../hardware/util/mag_decl.h"
#include "../control/position/navi.h"
#include "../control/position/att_ctrl.h"
#include "../control/position/u_ctrl.h"
#include "../control/position/yaw_ctrl.h"
#include "../control/speed/u_speed.h"
#include "../control/speed/piid.h"
#include "../control/speed/ne_speed.h"
#include "../state/motors_state.h"
#include "../force_opt/force_opt.h"
#include "../force_opt/att_thrust.h"
#include "../flight_logic/flight_logic.h"


static float *rpm_square = NULL;
static float *setpoints = NULL;
static void *blackbox_socket = NULL;
static msgpack_sbuffer *msgpack_buf;
static msgpack_packer *pk;
static float mag_decl = 0.0f;
static gps_data_t gps_data;
static gps_rel_data_t gps_rel_data = {0.0, 0.0, 0.0};
static calibration_t gyro_cal;
static gps_util_t gps_util;
static interval_t gyro_move_interval;
static int init = 0;
static body_to_world_t *btw;
static flight_state_t flight_state;


static char *blackbox_spec[] = {"dt",                      /*  1      */
   "gyro_x", "gyro_y", "gyro_z",                           /*  2 -  4 */
   "sp_rate_x", "sp_rate_y", "sp_rate_z",                  /*  5 -  7 */
   "acc_x", "acc_y", "acc_z",                              /*  8 - 10 */
   "mag_x", "mag_y", "mag_z",                              /* 11 - 13 */
   //"q0", "q1", "q2", "q3",                                 /* 14 - 17 */
   "yaw", "pitch", "roll",                                 /* 18 - 20 */
   "acc_e", "acc_n", "acc_u",                              /* 21 - 23 */
   "raw_e", "raw_n", "raw_ultra_u", "raw_baro_u",          /* 24 - 27 */
   "pos_e", "pos_n", "pos_ultra_u", "pos_baro_u",          /* 28 - 31 */
   "speed_e",  "pos_n", "speed_ultra_u", "pos_ultra_u",    /* 32 - 35 */
   "yaw_sp", "pitch_sp", "roll_sp",                        /* 36 - 38 */
   "flight_state", "rc_valid",                             /* 39 - 40 */
   "rc_pitch", "rc_roll", "rc_yaw", "rc_gas", "rc_switch", /* 41 - 45 */
   "sensor_status", "init", "voltage"                      /* 46 - 48 */
};


typedef union
{
   struct
   {
      float gas;   /* [N] */
      float roll;  /* rad/s */
      float pitch; /* rad/s */
      float yaw;   /* rad/s */
   };
   float vec[4];
}
f_local_t;


static int marg_err = 0;
static pos_in_t pos_in;


static int gyro_moved(vec3_t *gyro)
{
   FOR_N(i, 3)
   {
      if (fabs(gyro->vec[i]) > 0.15)
      {
         return 1;   
      }
   }
   return 0;
}


void main_init(int override_hw)
{
   /* init data structures: */
   memset(&pos_in, 0, sizeof(pos_in_t));

   /* init SCL subsystem: */
   syslog(LOG_INFO, "initializing signaling and communication link (SCL)");
   if (scl_init("pilot") != 0)
   {
      syslog(LOG_CRIT, "could not init scl module");
      die();
   }
   
   /* init params subsystem: */
   syslog(LOG_INFO, "initializing opcd interface");
   opcd_params_init("pilot.", 1);
   
   /* initialize logger: */
   syslog(LOG_INFO, "opening logger");
   if (logger_open() != 0)
   {
      syslog(LOG_CRIT, "could not open logger");
      die();
   }
   syslog(LOG_CRIT, "logger opened");
   sleep(1); /* give scl some time to establish
                a link between publisher and subscriber */
   
   LOG(LL_INFO, "initializing platform");
   if (arcade_quad_init(&platform, override_hw) < 0)
   {
      LOG(LL_ERROR, "could not initialize platform");
      die();
   }
   acc_mag_cal_init();
 
   force_opt_init(platform.imtx1, platform.imtx2, platform.imtx3, platform.rpm_square_min, platform.rpm_square_max);
   const size_t array_len = sizeof(float) * platform.n_motors;
   setpoints = malloc(array_len);
   ASSERT_NOT_NULL(setpoints);
   memset(setpoints, 0, array_len);
   rpm_square = malloc(array_len);
   ASSERT_NOT_NULL(rpm_square);
   memset(rpm_square, 0, array_len);

   LOG(LL_INFO, "initializing model/controller");
   pos_init();
   ne_speed_ctrl_init();
   att_ctrl_init();
   yaw_ctrl_init();
   u_speed_init(platform.mass_kg * 10.0f / platform.max_thrust_n);
   navi_init();

   LOG(LL_INFO, "initializing command interface");
   cmd_init();

   motors_state_init(0.12f, 0.8f);
   blackbox_socket = scl_get_socket("blackbox");

   /* send blackbox header: */
   msgpack_buf = msgpack_sbuffer_new();
   pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   msgpack_pack_array(pk, ARRAY_SIZE(blackbox_spec));
   FOR_EACH(i, blackbox_spec)
   {
      size_t len = strlen(blackbox_spec[i]);
      msgpack_pack_raw(pk, len);
      msgpack_pack_raw_body(pk, blackbox_spec[i], len);
   }
   scl_copy_send_dynamic(blackbox_socket, msgpack_buf->data, msgpack_buf->size);

   /* init flight logic: */
   flight_logic_init();

   /* init calibration data: */
   cal_init(&gyro_cal, 3, 500);
   btw = body_to_world_create();

   cal_ahrs_init(10.0f, 5.0f * REALTIME_PERIOD, 0.02f);
   gps_util_init(&gps_util);
   flight_state_init(50, 30, 4.0, 150.0, 1.3);
   
   piid_init(REALTIME_PERIOD);

   interval_init(&gyro_move_interval);
   gps_data_init(&gps_data);
   LOG(LL_INFO, "entering main loop");
}


void main_step(float dt,
               marg_data_t *marg_data,
               gps_data_t *gps_data,
               float ultra,
               float baro,
               float voltage,
               float channels[MAX_CHANNELS],
               uint16_t sensor_status,
               int override_hw)
{
   
   /* read sensor data and calibrate sensors: */
   pos_in.dt = dt;
   pos_in.ultra_z = ultra;
   pos_in.baro_z = baro;
   
   if (!(sensor_status & MARG_VALID))
   {
      marg_err += 1;
      if (marg_err > 5)
      {
         /* we are in serious trouble */
         memset(setpoints, 0, sizeof(float) * platform.n_motors);
         platform_write_motors(setpoints);
         die();
      }
      goto out;
   }
   marg_err = 0;
   

   /* perform gyro calibration: */
   if (cal_sample_apply(&gyro_cal, &marg_data->gyro.vec[0]) == 0 && gyro_moved(&marg_data->gyro))
   {
      if (interval_measure(&gyro_move_interval) > 1.0)
         LOG(LL_ERROR, "gyro moved during calibration, retrying");
      cal_reset(&gyro_cal);
   }

   if (sensor_status & GPS_VALID)
   {
      gps_util_update(&gps_rel_data, &gps_util, gps_data);
      pos_in.pos_e = gps_rel_data.de;
      pos_in.pos_n = gps_rel_data.dn;
      printf("n: %f e: %f\n", pos_in.pos_n, pos_in.pos_e);
      ONCE(mag_decl = mag_decl_lookup(gps_data->lat, gps_data->lon);
           gps_start_set(gps_data);
           LOG(LL_ERROR, "declination lookup yields: %f", mag_decl));
   }

   /* acc/mag calibration: */
   acc_mag_cal_apply(&marg_data->acc, &marg_data->mag);

   /* perform sensor data fusion: */
   euler_t euler;
   int ahrs_state = cal_ahrs_update(&euler, marg_data, dt);
   if (ahrs_state < 0 || !cal_complete(&gyro_cal))
      goto out;
   
   ONCE(init = 1; LOG(LL_DEBUG, "system initialized; orientation = yaw: %f pitch: %f roll: %f", euler.yaw, euler.pitch, euler.roll));
   
   /* local ACC to global ACC rotation: */
   body_to_world_transform(btw, &pos_in.acc, &euler, &marg_data->acc);

   /* compute next 3d position estimate: */
   pos_t pos_estimate;
   pos_update(&pos_estimate, &pos_in);
   flight_state = flight_state_update(&marg_data->acc.vec[0], pos_estimate.ultra_z.pos);
   
   /* execute flight logic (sets cm_x parameters used below): */
   flight_logic_run(sensor_status, channels, euler.yaw);
   
   /* RUN U POSITION AND SPEED CONTROLLER: */
   float u_err = 0.0f;
   float u_speed_sp = 0.0f;
   if (cm_u_is_pos())
   {
      if (cm_u_is_baro_pos())
         u_err = cm_u_setp() - pos_estimate.baro_z.pos;
      else
         u_err = cm_u_setp() - pos_estimate.ultra_z.pos;
      u_speed_sp = u_ctrl_step(u_err);
   }
   
   if (cm_u_is_spd())
      u_speed_sp = cm_u_setp();
   
   float f_d = u_speed_step(u_speed_sp, pos_estimate.baro_z.speed, dt);
   if (cm_u_is_acc())
      f_d = cm_u_setp();

   f_d = fmin(f_d, cm_u_acc_limit());
   f_d *= platform.max_thrust_n;

   vec2_t speed_sp = {{0.0f, 0.0f}};
   if (cm_att_is_gps_pos())
   {
      navi_set_dest(cm_att_setp());
      navi_run(&speed_sp, &pos_estimate.ne_pos, dt); /* attitude navigation control */
   }

   if (cm_att_is_gps_spd())
      speed_sp = cm_att_setp(); /* direct attitude speed control */

   /* RUN ATT NORTH/EAST SPEED CONTROLLER: */
   vec2_t f_ne;
   ne_speed_ctrl_run(&f_ne, &speed_sp, dt, &pos_estimate.ne_speed, euler.yaw);
   vec3_t f_ned = {{f_ne.vec[0], f_ne.vec[1], f_d}};

   vec2_t pitch_roll_sp;
   float thrust;
   att_thrust_calc(&pitch_roll_sp, &thrust, &f_ned, platform.max_thrust_n, 0);

   if (cm_att_is_angle())
      pitch_roll_sp = cm_att_setp(); /* direct attitude angle control */
 
   /* RUN ATT ANGLE CONTROLLER: */
   vec2_t att_err;
   vec2_t pitch_roll_speed = {{marg_data->gyro.y, marg_data->gyro.x}};
   vec2_t pitch_roll_ctrl;
   vec2_t pitch_roll = {{euler.pitch, euler.roll}};
   att_ctrl_step(&pitch_roll_ctrl, &att_err, dt, &pitch_roll, &pitch_roll_speed, &pitch_roll_sp);
 
   float piid_sp[3] = {0.0f, 0.0f, 0.0f};
   piid_sp[PIID_PITCH] = pitch_roll_ctrl.x;
   piid_sp[PIID_ROLL] = pitch_roll_ctrl.y;
 
   if (cm_att_is_rate())
   {
      /* direct attitude rate control */
      vec2_t setp = cm_att_setp();
      piid_sp[PIID_PITCH] = setp.x;
      piid_sp[PIID_ROLL] = setp.y;
   }

   /* RUN YAW CONTROLLER: */
   float yaw_speed_sp, yaw_err;
   if (cm_yaw_is_pos())
   {
      /* yaw position control */
      float sp = cm_yaw_setp();
      yaw_speed_sp = yaw_ctrl_step(&yaw_err, sp, euler.yaw, marg_data->gyro.z, dt);
   }
   else
      yaw_speed_sp = cm_yaw_setp(); /* direct yaw speed control */
   
   piid_sp[PIID_YAW] = yaw_speed_sp;

   /* RUN STABLIZING PIID CONTROLLER: */
   f_local_t f_local = {{thrust, 0.0f, 0.0f, 0.0f}};
   piid_run(&f_local.vec[1], marg_data->gyro.vec, piid_sp);

   /* computation of rpm ^ 2 out of the desired forces */
   inv_coupling_calc(&platform.inv_coupling, rpm_square, f_local.vec);
   
   /* compute motor set points out of rpm ^ 2: */
   piid_int_enable(platform_ac_calc(setpoints, cm_motors_enabled(), voltage, rpm_square));
   if (!cm_motors_enabled())
   {
      memset(setpoints, 0, sizeof(float) * platform.n_motors);
      piid_reset(); /* reset piid integrators so that we can move the device manually */
      att_ctrl_reset();
   }
   else
   {
      /* notify any cpu-consuming applications to stop processing */
      
      goto out;
   }

   /* write motors: */
   if (!override_hw)
   {
      //platform_write_motors(setpoints);
   }

   /* set monitoring data: */
   mon_data_set(pos_estimate.ne_pos.n - navi_get_dest_n(),
                pos_estimate.ne_pos.e - navi_get_dest_e(),
                u_err, yaw_err);
 
out:
   /* publish blackbox data; this is also executed in case of a sensor error: */
   msgpack_sbuffer_clear(msgpack_buf);
   msgpack_pack_array(pk, ARRAY_SIZE(blackbox_spec));
   #define PACKI(val) msgpack_pack_int(pk, val) /* pack integer */
   #define PACKF(val) msgpack_pack_float(pk, val) /* pack float */
   #define PACKFV(ptr, n) FOR_N(i, n) PACKF(ptr[i]) /* pack float vector */
   PACKF(dt);
   PACKFV(marg_data->gyro.vec, 3);
   PACKFV(piid_sp, 3);
   PACKFV(marg_data->acc.vec, 3);
   PACKFV(marg_data->mag.vec, 3);
   PACKFV(euler.vec, 3);
   PACKFV(pos_in.acc.vec, 3);
   PACKF(pos_in.pos_e); PACKF(pos_in.pos_n);
   PACKF(pos_in.ultra_z); PACKF(pos_in.baro_z);
   PACKFV(pos_estimate.ne_pos.vec, 2);
   PACKF(pos_estimate.ultra_z.pos); PACKF(pos_estimate.baro_z.pos);
   PACKFV(pos_estimate.ne_speed.vec, 2);
   PACKF(pos_estimate.ultra_z.speed); PACKF(pos_estimate.baro_z.speed);
   PACKF(0.0f);
   PACKFV(pitch_roll_sp.vec, 2);
   PACKI(flight_state);
   int rc_valid = 0;
   PACKI(rc_valid);
   PACKFV(channels, 5);
   PACKI(sensor_status);
   PACKI(init);
   PACKF(voltage);
   scl_copy_send_dynamic(blackbox_socket, msgpack_buf->data, msgpack_buf->size);
}

