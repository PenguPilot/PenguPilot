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
  
 ARCADE Quadrotor Platform

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology
 Copyright (C) 2012 Alexander Barth, Ilmenau University of Technology
 Copyright (C) 2012 Benjamin Jahn, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <stddef.h>
#include <unistd.h>
#include <malloc.h>

#include <util.h>
#include "inv_coupling.h"
#include "platform.h"
#include "../util/logger/logger.h"

/* hardware includes: */
#include "../hardware/bus/i2c/i2c.h"
#include "../hardware/drivers/scl_gps/scl_gps.h"
#include "../hardware/drivers/i2cxl_reader/i2cxl_reader.h"
#include "../hardware/drivers/ms5611_reader/ms5611_reader.h"
#include "../hardware/drivers/rc_dsl_reader/rc_dsl_reader.h"
#include "../hardware/drivers/holger_blmc/holger_blmc.h"
#include "../hardware/drivers/holger_blmc/force2twi.h"
#include "../hardware/drivers/scl_voltage/scl_voltage.h"
#include "../hardware/util/rc_channels.h"
#include "drotek_marg2.h"

/* optimizer includes: */
#include "../control/util/cvxgen/solver.h"

#define CONVEXOPT_MIN_GROUND_DIST 1.0f

#define RPM_MIN 1340.99f
#define RPM_MAX 7107.6f
#define RPM_SQUARE_MIN (RPM_MIN * RPM_MIN)
#define RPM_SQUARE_MAX (RPM_MAX * RPM_MAX)

/* arm length */
#define CTRL_L (0.2025f)

/* F = c * rpm ^ 2 */
#define F_C (1.5866e-007f)

/* tau = d * rpm ^ 2 */
#define F_D (4.0174e-009f)

/* number of motors we can control: */
#define N_MOTORS (4)

#define IMTX1 (1.0f / (4.0f * F_C))
#define IMTX2 (1.0f / (2.0f * F_C * CTRL_L))
#define IMTX3 (1.0f / (4.0f * F_D))


/* inverse coupling matrix for ARCADE quadrotor: */
static float inv_coupling_matrix[4 * N_MOTORS] =
{         /* gas    pitch    roll   yaw */
   /* m0 */ IMTX1,   0.0f, -IMTX2,  IMTX3,
   /* m1 */ IMTX1,   0.0f,  IMTX2,  IMTX3,
   /* m2 */ IMTX1, -IMTX2,   0.0f, -IMTX3,
   /* m3 */ IMTX1,  IMTX2,   0.0f, -IMTX3
};

                                     /* m0    m1    m2    m3 */
static uint8_t motor_addrs[N_MOTORS] = {0x29, 0x2a, 0x2b, 0x2c};
static i2c_bus_t i2c_3;
static uint8_t *motor_setpoints = NULL;
static deadzone_t deadzone;
static rc_channels_t rc_channels;
static uint8_t channel_mapping[MAX_CHANNELS] =  {0, 1, 3, 2, 4}; /* pitch: 0, roll: 1, yaw: 3, gas: 2, switch: 4 */
static float channel_scale[MAX_CHANNELS] =  {1.0f, -1.0f, -1.0f, 1.0f, 1.0f};
static drotek_marg2_t marg;
static inv_coupling_t inv_coupling; 
static float *rpm_square;


static void convex_opt_init(void);
static void convex_opt_run(float forces[4]);

static int rpm_stable_count = 0;
static int rpm_stable = 100;
static float rpm_inval_epsilon = 10;



static int write_motors(int enabled, float forces[4], float voltage)
{
   float opt_forces[4];
   memcpy(opt_forces, forces, sizeof(float) * 4);
   //convex_opt_run(opt_forces);

   float ground_dist = CONVEXOPT_MIN_GROUND_DIST - 1.0f;
   if (i2cxl_reader_get_alt(&ground_dist) == 0)
   {
      if (ground_dist > CONVEXOPT_MIN_GROUND_DIST)
      {
         memcpy(forces, opt_forces, sizeof(float) * 4);
      }
   }
   /* computation of rpm ^ 2 out of the desired forces */
   inv_coupling_calc(&inv_coupling, rpm_square, forces);
   int status = 0;
   if (enabled)
   {
      if (force2twi_calc(motor_setpoints, voltage, rpm_square, N_MOTORS))
         status |= MOTORS_INT_ENABLE;
   }
   else
   {
      memset(motor_setpoints, HOLGER_I2C_OFF, N_MOTORS);
   }

   /* write motors and read rpm: */
   uint8_t rpm[4];
   holger_blmc_write_read(motor_setpoints, rpm);
   
   /* check if rpm readings match our expectations: */
   FOR_N(i, N_MOTORS)
   {
      if (motor_setpoints[i] < HOLGER_I2C_MIN)
         rpm_stable_count = 0;
      else if (fabs(1000.0f * rpm[i] - sqrtf(rpm_square[i])) > rpm_inval_epsilon)
         rpm_stable_count = 0;
   }
   if (rpm_stable_count++ == rpm_stable)
   {
      rpm_stable_count = rpm_stable;
      status |= MOTORS_RPM_STABLE;
   }

   return status;
}


static int read_rc(float channels[MAX_CHANNELS])
{
   float dsl_channels[RC_DSL_CHANNELS];
   int ret = rc_dsl_reader_get(dsl_channels);
   int c;
   for (c = 0; c < MAX_CHANNELS; c++)
   {
      channels[c] = rc_channels_get(&rc_channels, dsl_channels, c);
   }
   return ret;
}


static int read_marg(marg_data_t *marg_data)
{
   return drotek_marg2_read(marg_data, &marg);   
}


int arcade_quadro_init(platform_t *plat)
{
   ASSERT_ONCE();
   THROW_BEGIN();

   /* local initializations: */
   convex_opt_init();
   
   LOG(LL_INFO, "setting platform parameters");
   plat->param.max_thrust_n = 28.0f;
   plat->param.mass_kg = 0.95f;

   LOG(LL_INFO, "initializing i2c bus");
   THROW_ON_ERR(i2c_bus_open(&i2c_3, "/dev/i2c-3"));

   LOG(LL_INFO, "initializing MARG sensor cluster");
   THROW_ON_ERR(drotek_marg2_init(&marg, &i2c_3));
   plat->read_marg = read_marg;

   LOG(LL_INFO, "initializing i2cxl sonar sensor");
   THROW_ON_ERR(i2cxl_reader_init(&i2c_3));
   plat->read_ultra = i2cxl_reader_get_alt;
   
   LOG(LL_INFO, "initializing ms5611 barometric pressure sensor");
   THROW_ON_ERR(ms5611_reader_init(&i2c_3));
   plat->read_baro = ms5611_reader_get_alt;

   LOG(LL_INFO, "initializing inverse coupling matrix");
   inv_coupling_init(&inv_coupling, N_MOTORS, inv_coupling_matrix);

   /* set-up motors driver: */
   LOG(LL_INFO, "initializing motor drivers");
   rpm_square = malloc(N_MOTORS * sizeof(float));
   motor_setpoints = malloc(sizeof(uint8_t) * N_MOTORS);
   memset(motor_setpoints, 0, sizeof(uint8_t) * N_MOTORS);
   holger_blmc_init(&i2c_3, motor_addrs, N_MOTORS);
   plat->write_motors = write_motors;
 
   /* set-up gps driver: */
   scl_gps_init();
   plat->read_gps = scl_gps_read;

   /* set-up dsl reader: */
   LOG(LL_INFO, "initializing DSL reader");
   if (rc_dsl_reader_init() < 0)
   {
      LOG(LL_ERROR, "could not initialize dsl reader");
      exit(1);
   }
   deadzone_init(&deadzone, 0.01f, 1.0f, 1.0f);
   rc_channels_init(&rc_channels, channel_mapping, channel_scale, &deadzone);
   plat->read_rc = read_rc;

   if (scl_voltage_init() < 0)
   {
      LOG(LL_ERROR, "could not initialize voltage reader");
      exit(1);
   }
   plat->read_voltage = scl_voltage_read;
 
   LOG(LL_INFO, "arcade_quadro platform initialized");
   THROW_END();
}



/* convex optimization stuff: */

Vars vars;
Params params;
Workspace work;
Settings settings;


static void convex_opt_init(void)
{
  set_defaults();
  setup_indexing();
  
  /* G-Matrix */
  params.G[0] = 1e-4;
  params.G[4] = 0;
  params.G[8] = 0;
  params.G[12] = 0;

  params.G[1] = 0;
  params.G[5] = 1.0;
  params.G[9] = 0;
  params.G[13] = 0;

  params.G[2] = 0;
  params.G[6] = 0;
  params.G[10] = 1.0;
  params.G[14] = 0;

  params.G[3] = 0;
  params.G[7] = 0;
  params.G[11] = 0;
  params.G[15] = 2.0;

  /* A-Matrix */
  params.A[0] = (float)((double)IMTX1)/((double)((double)RPM_SQUARE_MAX));
  params.A[8] = 0.0;
  params.A[16] = -(float)((double)IMTX2)/((double)RPM_SQUARE_MAX);
  params.A[24] = (float)((double)IMTX3)/((double)RPM_SQUARE_MAX);

  params.A[1] = (float)((double)IMTX1)/((double)RPM_SQUARE_MAX);
  params.A[9] = 0.0;
  params.A[17] = (float)((double)IMTX2)/((double)RPM_SQUARE_MAX);
  params.A[25] = (float)((double)IMTX3)/((double)RPM_SQUARE_MAX);

  params.A[2] = (float)((double)IMTX1)/((double)RPM_SQUARE_MAX);
  params.A[10] = -(float)((double)IMTX2)/((double)RPM_SQUARE_MAX);
  params.A[18] = 0.0;;
  params.A[26] = -((float)(double)IMTX3)/((double)RPM_SQUARE_MAX);

  params.A[3] = (float)((double)IMTX1)/((double)RPM_SQUARE_MAX);
  params.A[11] = (float)((double)IMTX2)/((double)RPM_SQUARE_MAX);
  params.A[19] = 0.0;
  params.A[27] = -(float)((double)IMTX3)/((double)RPM_SQUARE_MAX);

  params.A[4]  = -params.A[0];
  params.A[12] = -params.A[8];
  params.A[20] = -params.A[16];
  params.A[28] = -params.A[24];

  params.A[5]  = -params.A[1];
  params.A[13] = -params.A[9];
  params.A[21] = -params.A[17];
  params.A[29] = -params.A[25];

  params.A[6]  = -params.A[2];
  params.A[14] = -params.A[10];
  params.A[22] = -params.A[18];
  params.A[30] = -params.A[26];

  params.A[7]  = -params.A[3];
  params.A[15] = -params.A[11];
  params.A[23] = -params.A[19];
  params.A[31] = -params.A[27];

  params.b[0] = 1.0;
  params.b[1] = 1.0;
  params.b[2] = 1.0;
  params.b[3] = 1.0;
  params.b[4] = -(float)((double)RPM_SQUARE_MIN)/((double)RPM_SQUARE_MAX);
  params.b[5] = -(float)((double)RPM_SQUARE_MIN)/((double)RPM_SQUARE_MAX);
  params.b[6] = -(float)((double)RPM_SQUARE_MIN)/((double)RPM_SQUARE_MAX);
  params.b[7] = -(float)((double)RPM_SQUARE_MIN)/((double)RPM_SQUARE_MAX);

  settings.verbose = 0;  // disable output of solver progress.
  settings.max_iters = 10;  // reduce the maximum iteration count, from 25.
  settings.eps = 1e-6;;  // reduce the required objective tolerance, from 1e-6.
  settings.resid_tol = 1e-6;  // reduce the required residual tolerances, from 1e-4.
  settings.better_start = 1;
}


static void convex_opt_run(float forces[4])
{
  params.xd[0] = forces[0];
  params.xd[1] = forces[1];
  params.xd[2] = forces[2];
  params.xd[3] = forces[3];

  solve();

  forces[0] = vars.x[0];
  forces[1] = vars.x[1];
  forces[2] = vars.x[2];
  forces[3] = vars.x[3];
}

