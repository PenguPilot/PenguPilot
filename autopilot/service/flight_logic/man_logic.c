
#include <threadsafe_types.h>
#include <opcd_interface.h>
#include <util.h>

#include "man_logic.h"
#include "../filters/filter.h"
#include "../hardware/util/calibration.h"
#include "../util/logger/logger.h"
#include "../util/math/conv.h"
#include "../util/time/interval.h"
#include "../main_loop/main_loop.h"
#include "../main_loop/control_mode.h"


typedef enum
{
   MAN_SPORT,
   MAN_RELAXED,
   MAN_NOVICE
}
man_mode_t;
man_mode_t last_mode = -1;


static tsfloat_t stick_pitch_roll_p;
static tsfloat_t stick_pitch_roll_angle_max;
static tsfloat_t stick_yaw_p;
static tsfloat_t vert_speed_max;
static tsfloat_t horiz_speed_max;

static bool vert_pos_locked = true;
static bool horiz_pos_locked = true;
static calibration_t rc_cal;
static Filter1 rc_valid_filter;
static interval_t rc_lost_interval;


void man_logic_init(void)
{
   interval_init(&rc_lost_interval);
   cal_init(&rc_cal, 3, 500);
   filter1_lp_init(&rc_valid_filter, 0.5, REALTIME_PERIOD, 1);

   opcd_param_t params[] =
   {
      {"pitch_roll_p", &stick_pitch_roll_p},
      {"pitch_roll_angle_max", &stick_pitch_roll_angle_max},
      {"vert_speed_max", &vert_speed_max},
      {"horiz_speed_max", &horiz_speed_max},
      {"yaw_p", &stick_yaw_p},
      OPCD_PARAMS_END
   };
   opcd_params_apply("sticks.", params);
}


static man_mode_t channel_to_man_mode(float sw)
{
   float a = 0.333;
   float b = 0.666;

   if (sw <= a)
   {
      return MAN_SPORT;   
   }
   else if (sw > a && sw < b)
   {
      return MAN_RELAXED;
   }
   return MAN_NOVICE;
}


static void handle_mode_update(man_mode_t mode)
{
   if (last_mode != mode)
   {
      LOG(LL_INFO, "switching manual mode to: %d", mode);
      last_mode = mode;   
   }
}


static void set_vertical_spd_or_pos(float gas_stick, float u_baro_pos)
{
   if (fabs(gas_stick) > 0.2)
   {
      float vmax = tsfloat_get(&vert_speed_max);
      cm_u_set_spd(gas_stick * vmax);
      vert_pos_locked = false;
   }
   else if (!vert_pos_locked)
   {
      vert_pos_locked = true;
      cm_u_set_baro_pos(u_baro_pos);
   }
}


static void set_pitch_roll_rates(float pitch, float roll)
{
   float p = tsfloat_get(&stick_pitch_roll_p);
   vec2_t pitch_roll = {{-p * pitch, p * roll}};
   cm_att_set_rates(pitch_roll);
}


static void set_horizontal_spd_or_pos(float pitch, float roll, float yaw, vec2_t *ne_gps_pos)
{
   float vmax_sqrt = sqrt(tsfloat_get(&stick_pitch_roll_p));
   if (sqrt(pitch * pitch + roll * roll) > 0.01)
   {
      vec2_t local_spd_sp = {{vmax_sqrt * pitch, vmax_sqrt * roll}};
      vec2_t global_spd_sp;
      vec2_rotate(&global_spd_sp, &local_spd_sp, yaw);
      cm_att_set_gps_spd(global_spd_sp);
      horiz_pos_locked = false;
   }
   else if (!horiz_pos_locked)
   {
      horiz_pos_locked = true;
      cm_att_set_gps_pos(*ne_gps_pos);   
   }
}


void set_att_angles(float pitch, float roll)
{
   float a = deg2rad(tsfloat_get(&stick_pitch_roll_angle_max));
   vec2_t pitch_roll = {{a * pitch, -a * roll}};
   cm_att_set_rates(pitch_roll);
}


static void emergency_landing(bool gps_valid, vec2_t *ne_gps_pos, float u_ultra_pos)
{
   cm_u_set_spd(-0.5f);
   if (gps_valid)
      set_horizontal_spd_or_pos(0.0f, 0.0f, 0.0f, ne_gps_pos);
   else
      set_pitch_roll_rates(0.0f, 0.0f);
   
   if (u_ultra_pos < 0.3f)
      cm_disable_motors_persistent();
}


void man_logic_run(uint16_t sensor_status, float channels[MAX_CHANNELS], float yaw, vec2_t *ne_gps_pos, float u_baro_pos, float u_ultra_pos)
{
   float rc_valid_f = (sensor_status & RC_VALID) ? 1.0f : 0.0f;
   filter1_run(&rc_valid_filter, &rc_valid_f, &rc_valid_f);
   if (rc_valid_f < 0.5f)
   {
      if (interval_measure(&rc_lost_interval) > 1.0f)
         emergency_landing(sensor_status & GPS_VALID, ne_gps_pos, u_ultra_pos);
      return;
   }
   interval_init(&rc_lost_interval);

   float cal_channels[3] = {channels[CH_PITCH], channels[CH_ROLL], channels[CH_YAW]};
   cal_sample_apply(&rc_cal, cal_channels);
   float pitch = cal_channels[0];
   float roll = cal_channels[1];
   float yaw_stick = cal_channels[2];
   float gas_stick = channels[CH_GAS];
   float sw_l = channels[CH_SWITCH_L];
   float sw_r = channels[CH_SWITCH_R];

   cm_enable_motors(sw_l > 0.5 ? true : false);

   cm_yaw_set_spd(yaw_stick); /* the only applied mode in manual operation */
   man_mode_t man_mode = channel_to_man_mode(sw_r);
   if (man_mode == MAN_NOVICE && (!(sensor_status & GPS_VALID)))
   {
      /* lost gps fix: switch to attitude control */
      man_mode = MAN_RELAXED;
   }
   handle_mode_update(man_mode);
   
   switch (man_mode)
   {
      case MAN_SPORT:
      {
         set_pitch_roll_rates(pitch, roll);
         cm_u_set_acc(gas_stick);
         break;
      }

      case MAN_RELAXED:
      {
         set_att_angles(pitch, roll);
         set_vertical_spd_or_pos(gas_stick - 0.5, u_baro_pos);
         break;
      }

      case MAN_NOVICE:
      {
         set_horizontal_spd_or_pos(pitch, roll, yaw, ne_gps_pos);
         set_vertical_spd_or_pos(gas_stick - 0.5, u_baro_pos);
         break;
      }
   }
}

