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
#include <threadsafe_types.h>
#include <physics.h>
#include <interval.h>

#include "mon.h"
#include "control_mode.h"
#include "main_loop.h"
#include "main_util.h"
#include "../interface/interface.h"
#include "../util/math/conv.h"
#include "../util/logger/logger.h"
#include "../estimators/cal_ahrs.h"
#include "../estimators/pos.h"
#include "../hardware/platform/platform.h"
#include "../hardware/platform/overo_quad.h"
#include "../hardware/platform/exynos_quad.h"
#include "../hardware/platform/pi_quad.h"
#include "../hardware/platform/inv_coupling.h"
#include "../hardware/util/acc_mag_cal.h"
#include "../hardware/util/cmc.h"
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
#include "../filters/filter.h"


static bool calibrate = false;
static float *rpm_square = NULL;
static float *setpoints = NULL;
static gps_data_t gps_data;
static gps_rel_data_t gps_rel_data = {0.0, 0.0, 0.0f, 0.0f};
static calibration_t gyro_cal;
static interval_t gyro_move_interval;
static int init = 0;
static Filter1 lp_filter;
static float acc_vec[3];


typedef union
{
   struct
   {
      float gas;   /* [N] */
      float roll;  /* rad/s */
      float pitch; /* rad/s */
      float yaw;   /* rad/s */
   };
   float ve[4];
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
   vec3_init(&pos_in.acc);

   /* init SCL subsystem: */
   syslog(LOG_INFO, "initializing signaling and communication link (SCL)");
   if (scl_init("autopilot") != 0)
   {
      syslog(LOG_CRIT, "could not init scl module");
      die();
   }
   
   /* init params subsystem: */
   syslog(LOG_INFO, "initializing opcd interface");
   opcd_params_init("autopilot.", 1);
   
   /* initialize logger: */
   syslog(LOG_INFO, "opening logger");
   if (logger_open() != 0)
   {
      syslog(LOG_CRIT, "could not open logger");
      die();
   }
   syslog(LOG_CRIT, "logger opened");
   
   LOG(LL_INFO, "initializing platform");
   
   char *plat_name = NULL;
   opcd_param_get("platform", &plat_name);
   if (strcmp(plat_name, "overo_quad") == 0)
   {
      if (overo_quad_init(&platform, override_hw) < 0)
      {
         LOG(LL_ERROR, "could not initialize platform");
         die();
      }
   }
   else if (strcmp(plat_name, "pi_quad") == 0)
   {
      if (pi_quad_init(&platform, override_hw) < 0)
      {
         LOG(LL_ERROR, "could not initialize platform");
         die();
      }
   }  
   else if (strcmp(plat_name, "exynos_quad") == 0)
   {
      if (exynos_quad_init(&platform, override_hw) < 0)
      {
         LOG(LL_ERROR, "could not initialize platform");
         die();
      }
   }
   else
   {
      LOG(LL_ERROR, "unknown platform: %s", plat_name);
      die();   
   }

   acc_mag_cal_init();
   cmc_init();
 
   const size_t array_len = sizeof(float) * platform.n_motors;
   setpoints = malloc(array_len);
   ASSERT_NOT_NULL(setpoints);
   memset(setpoints, 0, array_len);
   rpm_square = malloc(array_len);
   ASSERT_NOT_NULL(rpm_square);
   memset(rpm_square, 0, array_len);

   LOG(LL_INFO, "initializing model/controller");
   pos_init();
   ne_speed_ctrl_init(REALTIME_PERIOD);
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

   cal_ahrs_init();
   flight_state_init(50, 150, 4.0);
   
   piid_init(REALTIME_PERIOD);

   interval_init(&gyro_move_interval);
   gps_data_init(&gps_data);

   mag_decl_init();
   cal_init(&rc_cal, 3, 500);

   tsfloat_t acc_fg;
   opcd_param_t params[] =
   {
      {"acc_fg", &acc_fg},
      OPCD_PARAMS_END
   };
   opcd_params_apply("main.", params);
   filter1_lp_init(&lp_filter, tsfloat_get(&acc_fg), 0.005, 3);
   lp_filter.z[2] = G_CONSTANT;

   cm_init();
   mon_init();
   LOG(LL_INFO, "entering main loop");
}



void main_step(const float dt,
               const marg_data_t *marg_data,
               const gps_data_t *gps_data,
               const float ultra,
               const float baro,
               const float voltage,
               const float current,
               const float channels[MAX_CHANNELS],
               const uint16_t sensor_status,
               const bool override_hw)
{
   vec2_t ne_pos_err, ne_speed_sp, ne_spd_err;
   vec2_init(&ne_pos_err);
   vec2_init(&ne_speed_sp);
   vec2_init(&ne_spd_err);
   vec3_t mag_normal;
   vec3_init(&mag_normal);

   float u_pos_err = 0.0f;
   float u_spd_err = 0.0f;
   
   int bb_rate;
   if (calibrate)
   {
      ONCE(LOG(LL_INFO, "publishing calibration data; actuators disabled"));
      bb_rate = 20;
      goto out;
   }
   else
      bb_rate = 1;
   
   pos_in.dt = dt;
   pos_in.ultra_u = ultra;
   pos_in.baro_u = baro;
   if (!(sensor_status & MARG_VALID))
   {
      marg_err += 1;
      if (marg_err > 500)
      {
         /* we are in serious trouble */
         memset(setpoints, 0, sizeof(float) * platform.n_motors);
         platform_write_motors(setpoints);
         die();
      }
      goto out;
   }
   marg_err = 0;
   
   float cal_channels[MAX_CHANNELS];
   memcpy(cal_channels, channels, sizeof(cal_channels));
   if (sensor_status & RC_VALID)
   {
      /* apply calibration if remote control input is valid: */
      float cal_data[3] = {channels[CH_PITCH], channels[CH_ROLL], channels[CH_YAW]};
      cal_sample_apply(&rc_cal, cal_data);
      cal_channels[CH_PITCH] = cal_data[0];
      cal_channels[CH_ROLL] = cal_data[1];
      cal_channels[CH_YAW] = cal_data[2];
   }

   /* perform gyro calibration: */
   marg_data_t cal_marg_data;
   marg_data_init(&cal_marg_data);
   marg_data_copy(&cal_marg_data, marg_data);
   if (cal_sample_apply(&gyro_cal, cal_marg_data.gyro.ve) == 0 && gyro_moved(&marg_data->gyro))
   {
      if (interval_measure(&gyro_move_interval) > 1.0)
         LOG(LL_ERROR, "gyro moved during calibration, retrying");
      cal_reset(&gyro_cal);
   }

   /* update relative gps position, if we have a fix: */
   float mag_decl;
   if (sensor_status & GPS_VALID)
   {
      gps_util_update(&gps_rel_data, gps_data);
      pos_in.pos_n = gps_rel_data.dn;
      pos_in.pos_e = gps_rel_data.de;
      pos_in.speed_n = gps_rel_data.speed_n;
      pos_in.speed_e = gps_rel_data.speed_e;
      ONCE(gps_start_set(gps_data));
      mag_decl = mag_decl_get();
   }
   else
   {
      pos_in.pos_n = 0.0f;   
      pos_in.pos_e = 0.0f; 
      pos_in.speed_n = 0.0f;
      pos_in.speed_e = 0.0f;
      mag_decl = 0.0f;
   }

   /* apply acc/mag calibration: */
   acc_mag_cal_apply(&cal_marg_data.acc, &cal_marg_data.mag);
   vec_copy(&mag_normal, &cal_marg_data.mag);

   /* apply current magnetometer compensation: */
   cmc_apply(&cal_marg_data.mag, current);

   /* determine flight state: */
   bool flying = flight_state_update(&cal_marg_data.acc.ve[0]);
   if (!flying && pos_in.ultra_u == 7.0)
      pos_in.ultra_u = 0.2;
   
   /* compute orientation estimate: */
   euler_t euler;
   int ahrs_state = cal_ahrs_update(&euler, &cal_marg_data, mag_decl, dt);
   if (ahrs_state < 0 || !cal_complete(&gyro_cal))
      goto out;
   ONCE(init = 1; LOG(LL_DEBUG, "system initialized; orientation = yaw: %f pitch: %f roll: %f", euler.yaw, euler.pitch, euler.roll));

   /* rotate local ACC measurements into global NEU reference frame: */
   vec3_t world_acc;
   vec3_init(&world_acc);
   body_to_neu(&world_acc, &euler, &cal_marg_data.acc);

   /* center global ACC readings: */
   filter1_run(&lp_filter, &world_acc.ve[0], &acc_vec[0]);
   FOR_N(i, 3)
      pos_in.acc.ve[i] = world_acc.ve[i] - acc_vec[i];

   /* compute next 3d position estimate using Kalman filters: */
   pos_t pos_est;
   vec2_init(&pos_est.ne_pos);
   vec2_init(&pos_est.ne_speed);
   pos_update(&pos_est, &pos_in);

   /* execute flight logic (sets cm_x parameters used below): */
   bool hard_off = false;
   bool motors_enabled = flight_logic_run(&hard_off, sensor_status, flying, cal_channels, euler.yaw, &pos_est.ne_pos, pos_est.baro_u.pos, pos_est.ultra_u.pos, platform.max_thrust_n, platform.mass_kg, dt);
   
   /* execute up position/speed controller(s): */
   float a_u = 0.0f;
   if (cm_u_is_pos())
   {
      if (cm_u_is_baro_pos())
         a_u = u_ctrl_step(&u_pos_err, cm_u_sp(), pos_est.baro_u.pos, pos_est.baro_u.speed, dt);
      else /* ultra pos */
         a_u = u_ctrl_step(&u_pos_err, cm_u_sp(), pos_est.ultra_u.pos, pos_est.ultra_u.speed, dt);
   }
   else if (cm_u_is_spd())
      a_u = u_speed_step(&u_spd_err, cm_u_sp(), pos_est.baro_u.speed, dt);
   else
      a_u = cm_u_sp();
 
   /* execute north/east navigation and/or read speed vector input: */
   if (cm_att_is_gps_pos())
   {
      vec2_t pos_sp;
      vec2_init(&pos_sp);
      cm_att_sp(&pos_sp);
      navi_run(&ne_speed_sp, &ne_pos_err, &pos_sp, &pos_est.ne_pos, dt);
   }
   else if (cm_att_is_gps_spd())
      cm_att_sp(&ne_speed_sp);

   /* execute north/east speed controller: */
   vec2_t a_ne;
   vec2_init(&a_ne);
   ne_speed_ctrl_run(&a_ne, &ne_spd_err, &ne_speed_sp, dt, &pos_est.ne_speed);
 
   vec3_t a_neu;
   vec3_set(&a_neu, a_ne.x, a_ne.y, a_u);
   vec3_t f_neu;
   vec3_init(&f_neu);
   vec_scalar_mul(&f_neu, &a_neu, platform.mass_kg); /* f[i] = a[i] * m, makes ctrl device-independent */

   float hover_force = platform.mass_kg * G_CONSTANT;
   f_neu.z += hover_force;

   /* execute NEU forces optimizer: */
   float thrust;
   vec2_t pr_pos_sp;
   vec2_init(&pr_pos_sp);
   att_thrust_calc(&pr_pos_sp, &thrust, &f_neu, euler.yaw, platform.max_thrust_n, 0);

   /* execute direct attitude angle control, if requested: */
   if (cm_att_is_angles())
      cm_att_sp(&pr_pos_sp);

   /* execute attitude angles controller: */
   vec2_t att_err;
   vec2_init(&att_err);
   vec2_t pr_spd;
   vec2_set(&pr_spd, -cal_marg_data.gyro.y, cal_marg_data.gyro.x);

   vec2_t pr_pos, pr_pos_ctrl;
   vec2_set(&pr_pos, -euler.pitch, euler.roll);
   vec2_init(&pr_pos_ctrl);
   att_ctrl_step(&pr_pos_ctrl, &att_err, dt, &pr_pos, &pr_spd, &pr_pos_sp);

   float piid_sp[3] = {0.0f, 0.0f, 0.0f};
   piid_sp[PIID_PITCH] = pr_pos_ctrl.ve[0];
   piid_sp[PIID_ROLL] = pr_pos_ctrl.ve[1];
 
   /* execute direct attitude rate control, if requested:*/
   if (cm_att_is_rates())
   {
      thrust = a_u + platform.mass_kg * G_CONSTANT;
      vec2_t rates_sp;
      vec2_init(&rates_sp);
      cm_att_sp(&rates_sp);
      piid_sp[PIID_PITCH] = rates_sp.ve[0];
      piid_sp[PIID_ROLL] = rates_sp.ve[1];
   }

   /* execute yaw position controller: */
   float yaw_speed_sp, yaw_err;
   if (cm_yaw_is_pos())
      yaw_speed_sp = yaw_ctrl_step(&yaw_err, cm_yaw_sp(), euler.yaw, cal_marg_data.gyro.z, dt);
   else
      yaw_speed_sp = cm_yaw_sp(); /* direct yaw speed control */
   piid_sp[PIID_YAW] = yaw_speed_sp;

   /* execute stabilizing PIID controller: */
   f_local_t f_local = {{thrust, 0.0f, 0.0f, 0.0f}};
   float piid_gyros[3] = {cal_marg_data.gyro.x, -cal_marg_data.gyro.y, cal_marg_data.gyro.z};
   piid_run(&f_local.ve[1], piid_gyros, piid_sp, dt);

   /* computate rpm ^ 2 out of the desired forces: */
   inv_coupling_calc(rpm_square, f_local.ve);

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
 
   /* handle special cases for motor setpoints: */
   if (motors_starting())
      FOR_N(i, platform.n_motors) setpoints[i] = platform.ac.min;
   if (hard_off || motors_stopping())
      FOR_N(i, platform.n_motors) setpoints[i] = platform.ac.off_val;
   
   /* write motors: */
   if (!override_hw)
   {
      platform_write_motors(setpoints);
   }

   /* set monitoring data: */
   mon_data_set(ne_pos_err.x, ne_pos_err.y, u_pos_err, yaw_err);

out:
   EVERY_N_TIMES(bb_rate, blackbox_record(dt, marg_data, gps_data, ultra, baro, voltage, current, channels, sensor_status, /* sensor inputs */
                          &ne_pos_err, u_pos_err, /* position errors */
                          &ne_spd_err, u_spd_err /* speed errors */,
                          &mag_normal));
}

