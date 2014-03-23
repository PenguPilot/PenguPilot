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

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <math.h>
#include <syslog.h>
#include <unistd.h>

#include <util.h>
#include <opcd_interface.h>
#include <scl.h>
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
#include "../platform/inv_coupling.h"
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
#include "../state/flight_state.h"
#include "../force_opt/att_thrust.h"
#include "../flight_logic/flight_logic.h"
#include "../blackbox/blackbox.h"


static bool calibrate = false;
static float *rpm_square = NULL;
static float *setpoints = NULL;
static float mag_decl = 0.0f;
static gps_data_t gps_data;
static gps_rel_data_t gps_rel_data = {0.0, 0.0, 0.0f, 0.0f};
static calibration_t gyro_cal;
static interval_t gyro_move_interval;
static int init = 0;
static body_to_neu_t *btn;
static float avg_acc[3] = {0, 0, 9.81};


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
static calibration_t rc_cal;


void main_init(int argc, char *argv[])
{
   bool override_hw = false;
   if (argc > 1)
   {
      if (strcmp(argv[1], "calibrate") == 0)
         calibrate = true;
      else
         override_hw = true;
   }

   /* init data structures: */
   memset(&pos_in, 0, sizeof(pos_in_t));
   
   /* read configuration: */
   opcd_param_t params[] =
   {
      OPCD_PARAMS_END
   };
   opcd_params_apply("main.", params);
 
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
   
   LOG(LL_INFO, "initializing platform");
   if (arcade_quad_init(&platform, override_hw) < 0)
   {
      LOG(LL_ERROR, "could not initialize platform");
      die();
   }
   acc_mag_cal_init();
 
   //force_opt_init(platform.imtx1, platform.imtx2, platform.imtx3, platform.rpm_square_min, platform.rpm_square_max);
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
   u_ctrl_init();
   u_speed_init();
   navi_init();

   LOG(LL_INFO, "initializing command interface");
   cmd_init();

   motors_state_init();
   blackbox_init();

   /* init flight logic: */
   flight_logic_init();

   /* init calibration data: */
   cal_init(&gyro_cal, 3, 1000);
   btn = body_to_neu_create();

   cal_ahrs_init(10.0f, 5.0f * REALTIME_PERIOD);
   flight_state_init(50, 150, 4.0);
   
   piid_init(REALTIME_PERIOD);

   interval_init(&gyro_move_interval);
   gps_data_init(&gps_data);

   cal_init(&rc_cal, 3, 500);

   mon_init();
   LOG(LL_INFO, "entering main loop");
}



void main_step(float dt,
               marg_data_t *marg_data,
               gps_data_t *gps_data,
               float ultra,
               float baro,
               float voltage,
               float current,
               float channels[MAX_CHANNELS],
               uint16_t sensor_status,
               bool override_hw)
{
   blackbox_record(dt, marg_data, gps_data, ultra, baro, voltage, current, channels, sensor_status);
   if (calibrate)
   {
      ONCE(LOG(LL_INFO, "publishing calibration data; actuators disabled"));
      return;
   }
   
   pos_in.dt = dt;
   pos_in.ultra_u = ultra;
   pos_in.baro_u = baro;
   
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
      return;
   }
   marg_err = 0;
   
   /* apply calibration if remote control input is valid: */
   if (sensor_status & RC_VALID)
   {
      float cal_channels[3] = {channels[CH_PITCH], channels[CH_ROLL], channels[CH_YAW]};
      cal_sample_apply(&rc_cal, cal_channels);
      channels[CH_PITCH] = cal_channels[0];
      channels[CH_ROLL] = cal_channels[1];
      channels[CH_YAW] = cal_channels[2];
   }

   /* perform gyro calibration: */
   if (cal_sample_apply(&gyro_cal, &marg_data->gyro.vec[0]) == 0 && gyro_moved(&marg_data->gyro))
   {
      if (interval_measure(&gyro_move_interval) > 1.0)
         LOG(LL_ERROR, "gyro moved during calibration, retrying");
      cal_reset(&gyro_cal);
   }

   /* update relative gps position, if we have a fix: */
   if (sensor_status & GPS_VALID)
   {
      gps_util_update(&gps_rel_data, gps_data);
      pos_in.pos_n = gps_rel_data.dn;
      pos_in.pos_e = gps_rel_data.de;
      pos_in.speed_n = gps_rel_data.speed_n;
      pos_in.speed_e = gps_rel_data.speed_e;
      /* compute declination and start gps position once: */
      ONCE(mag_decl = mag_decl_lookup(gps_data->lat, gps_data->lon);
           gps_start_set(gps_data);
           LOG(LL_INFO, "declination lookup yields: %f deg", rad2deg(mag_decl)));
   }

   /* acc/mag calibration: */
   acc_mag_cal_apply(&marg_data->acc, &marg_data->mag);
   bool flying = flight_state_update(&marg_data->acc.vec[0]);
   if (!flying && pos_in.ultra_u == 7.0)
      pos_in.ultra_u = 0.2;

   /* compute orientation estimate: */
   euler_t euler;
   int ahrs_state = cal_ahrs_update(&euler, marg_data, mag_decl, dt);
   if (ahrs_state < 0 || !cal_complete(&gyro_cal))
      return;
   ONCE(init = 1; LOG(LL_DEBUG, "system initialized; orientation = yaw: %f pitch: %f roll: %f", euler.yaw, euler.pitch, euler.roll));

   /* rotate local ACC measurements into global NEU reference frame: */
   vec3_t world_acc;
   body_to_neu(btn, &world_acc, &euler, &marg_data->acc);
   
   /* center global ACC readings: */
   FOR_N(i, 3)
   {
      pos_in.acc.vec[i] = world_acc.vec[i] - avg_acc[i];
      float a = 0.001;
      avg_acc[i] = avg_acc[i] * (1.0 - a) + world_acc.vec[i] * a;
   }

   /* compute next 3d position estimate using Kalman filters: */
   pos_t pos_est;
   pos_update(&pos_est, &pos_in);

   /* execute flight logic (sets cm_x parameters used below): */
   bool hard_off = 0;
   bool motors_enabled = flight_logic_run(&hard_off, sensor_status, flying, channels, euler.yaw, &pos_est.ne_pos, pos_est.baro_u.pos, pos_est.ultra_u.pos, platform.max_thrust_n, platform.mass_kg);
   
   /* execute up position/speed controller(s): */
   float u_err = 0.0f;
   float a_u = 0.0f;
   if (cm_u_is_pos())
   {
      if (cm_u_is_baro_pos())
      {
         a_u = u_ctrl_step(cm_u_setp(), pos_est.baro_u.pos, pos_est.baro_u.speed, dt);
         u_err = cm_u_setp() - pos_est.baro_u.pos;
      }
      else /* ultra pos */
      {
         u_err = cm_u_setp() - pos_est.ultra_u.pos;
         a_u = u_ctrl_step(cm_u_setp(), pos_est.ultra_u.pos, pos_est.ultra_u.speed, dt);
      }
   }
   else if (cm_u_is_spd())
      a_u = u_speed_step(cm_u_setp(), pos_est.baro_u.speed, dt);
   else
      a_u = cm_u_setp();
   
   /* execute north/east navigation and/or read speed vector input: */
   vec2_t ne_speed_sp = {{0.0f, 0.0f}};
   if (cm_att_is_gps_pos())
   {
      navi_set_dest(cm_att_setp());
      navi_run(&ne_speed_sp, &pos_est.ne_pos, dt);
   }
   else if (cm_att_is_gps_spd())
      ne_speed_sp = cm_att_setp();

   /* execute north/east speed controller: */
   vec2_t a_ne; /* acceleration vector in north/east direction */
   ne_speed_ctrl_run(&a_ne, &ne_speed_sp, dt, &pos_est.ne_speed);
   vec3_t a_neu = {{a_ne.x, a_ne.y, a_u}}, f_neu;
   vec3_mul_scalar(&f_neu, &a_neu, platform.mass_kg); /* f[i] = a[i] * m, makes ctrl device-independent */
   float hover_force = platform.mass_kg * 10.0f;
   f_neu.z += hover_force;

   /* execute NEU forces optimizer: */
   float thrust;
   vec2_t pitch_roll_sp;
   att_thrust_calc(&pitch_roll_sp, &thrust, &f_neu, euler.yaw, platform.max_thrust_n, 0);

   /* execute direct attitude angle control, if requested: */
   if (cm_att_is_angles())
      pitch_roll_sp = cm_att_setp();

   /* execute attitude angles controller: */
   vec2_t att_err;
   vec2_t pitch_roll_speed = {{-marg_data->gyro.y, marg_data->gyro.x}};
   vec2_t pitch_roll_ctrl;
   vec2_t pitch_roll = {{-euler.pitch, euler.roll}};
   att_ctrl_step(&pitch_roll_ctrl, &att_err, dt, &pitch_roll, &pitch_roll_speed, &pitch_roll_sp);
 
   float piid_sp[3] = {0.0f, 0.0f, 0.0f};
   piid_sp[PIID_PITCH] = pitch_roll_ctrl.vec[0];
   piid_sp[PIID_ROLL] = pitch_roll_ctrl.vec[1];
 
   /* execute direct attitude rate control, if requested:*/
   if (cm_att_is_rates())
   {
      vec2_t pitch_roll_rates_sp = cm_att_setp();
      piid_sp[PIID_PITCH] = pitch_roll_rates_sp.vec[0];
      piid_sp[PIID_ROLL] = pitch_roll_rates_sp.vec[1];
   }

   /* execute yaw position controller: */
   float yaw_speed_sp, yaw_err;
   if (cm_yaw_is_pos() && pos_est.ultra_u.pos > 1.0f)
      yaw_speed_sp = yaw_ctrl_step(&yaw_err, cm_yaw_setp(), euler.yaw, marg_data->gyro.z, dt);
   else
      yaw_speed_sp = cm_yaw_setp(); /* direct yaw speed control */
   piid_sp[PIID_YAW] = yaw_speed_sp;

   /* execute stabilizing PIID controller: */
   f_local_t f_local = {{thrust, 0.0f, 0.0f, 0.0f}};
   float piid_gyros[3] = {marg_data->gyro.x, -marg_data->gyro.y, marg_data->gyro.z};
   piid_run(&f_local.vec[1], piid_gyros, piid_sp, dt);

   /* computate rpm ^ 2 out of the desired forces: */
   inv_coupling_calc(rpm_square, f_local.vec);

   /* compute motor setpoints out of rpm ^ 2: */
   piid_int_enable(platform_ac_calc(setpoints, motors_spinning(), voltage, rpm_square));

   /* enables motors, if flight logic requests it: */
   motors_state_update(flying, dt, motors_enabled);
   
   /* reset controllers, if motors are not controllable: */
   if (!motors_controllable())
   {
      navi_reset();
      ne_speed_ctrl_reset();
      u_ctrl_reset();
      att_ctrl_reset();
      piid_reset();
   }
 
   /* handle special cases for motor setpoints (0.0f ... 1.0f): */
   if (motors_starting())
      FOR_N(i, platform.n_motors) setpoints[i] = 0.1f;
   if (hard_off || motors_stopping())
      FOR_N(i, platform.n_motors) setpoints[i] = 0.0f;
   
   /* write motors: */
   if (!override_hw)
   {
      platform_write_motors(setpoints);
   }

   /* set monitoring data: */
   mon_data_set(pos_est.ne_pos.n - navi_get_dest_n(),
                pos_est.ne_pos.e - navi_get_dest_e(),
                u_err, yaw_err);
}

