
#include <threadsafe_types.h>
#include <opcd_interface.h>

#include "man_logic.h"
#include "../filters/filter.h"
#include "../hardware/util/calibration.h"
#include "../util/logger/logger.h"
#include "../main_loop/main_loop.h"
#include "../main_loop/control_mode.h"


static tsfloat_t stick_pitch_roll_p;
static tsfloat_t stick_pitch_roll_angle_max;
static tsfloat_t stick_yaw_p;

static calibration_t rc_cal;
static Filter1 rc_valid_filter;


void man_logic_init(void)
{
   cal_init(&rc_cal, 3, 500);
   filter1_lp_init(&rc_valid_filter, 0.5, REALTIME_PERIOD, 1);

   /* read parameters: */
   opcd_param_t params[] =
   {
      {"pitch_roll_p", &stick_pitch_roll_p},
      {"pitch_roll_angle_max", &stick_pitch_roll_angle_max},
      {"yaw_p", &stick_yaw_p},
      OPCD_PARAMS_END
   };
   opcd_params_apply("sticks.", params);
}


typedef enum
{
   MAN_SPORT,
   MAN_RELAXED,
   MAN_NOVICE
}
man_mode_t;
man_mode_t last_mode = -1;

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


void man_logic_run(uint16_t sensor_status, float channels[MAX_CHANNELS], float yaw)
{
   float rc_valid_f = (sensor_status & RC_VALID) ? 1.0f : 0.0f;
   filter1_run(&rc_valid_filter, &rc_valid_f, &rc_valid_f);
   int rc_valid = rc_valid_f > 0.5f;
   if (!rc_valid)
      return;

   float cal_channels[3] = {channels[CH_PITCH], channels[CH_ROLL], channels[CH_YAW]};
   cal_sample_apply(&rc_cal, cal_channels);
   float pitch = cal_channels[0];
   float roll = cal_channels[1];
   float yaw_stick = cal_channels[2];
   float gas_stick = channels[CH_GAS];
   float sw_l = channels[CH_SWITCH_L];
   float sw_r = channels[CH_SWITCH_R];
   
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
         float p = tsfloat_get(&stick_pitch_roll_p);
         vec2_t pitch_roll = {{-p * pitch, p * roll}};
         cm_att_set_rates(pitch_roll);
         cm_u_acc_set(gas_stick);
         break;
      }

      case MAN_RELAXED:
      {
         float a = deg2rad(tsfloat_get(&stick_pitch_roll_angle_max));
         vec2_t pitch_roll = {{a * pitch, -a * roll}};
         cm_att_set_rates(pitch_roll);
         cm_u_spd_set(gas_stick);
         break;
      }

      case MAN_NOVICE:
      {
         float p = tsfloat_get(&stick_pitch_roll_p);
         vec2_t pitch_roll = {{p * pitch, p * roll}};
         vec2_t gps_spd_sp;
         vec2_rotate(&gps_spd_sp, &pitch_roll, yaw);
         cm_att_set_gps_spd(pitch_roll);
         cm_u_spd_set(gas_stick);
         break;
      }
   }
}

