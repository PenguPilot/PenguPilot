
#include <threadsafe_types.h>
#include <opcd_interface.h>

#include "man_logic.h"
#include "../filters/filter.h"
#include "../hardware/util/calibration.h"
#include "../util/math/conv.h"
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


void man_logic_run(uint16_t sensor_status, float channels[MAX_CHANNELS])
{
   float rc_valid_f = (sensor_status & RC_VALID) ? 1.0f : 0.0f;
   filter1_run(&rc_valid_filter, &rc_valid_f, &rc_valid_f);
   int rc_valid = rc_valid_f > 0.5f;
   float pitch, roll, yaw, gas, sw;
   if (!rc_valid)
   {
      pitch = 0.0f;
      roll = 0.0f;
      yaw = 0.0f;
      gas = 0.0f;
      sw = 0.0f;
   }
   else
   {
      float cal_channels[3] = {channels[CH_PITCH], channels[CH_ROLL], channels[CH_YAW]};
      cal_sample_apply(&rc_cal, cal_channels);
      pitch = cal_channels[0];
      roll = cal_channels[1];
      yaw = cal_channels[2];
      gas = channels[CH_GAS];
      sw = channels[CH_SWITCH];
   }
   
   man_mode_t man_mode = channel_to_man_mode(sw);
   switch (man_mode)
   {
      case MAN_SPORT:
      {
         cm_u_acc_set(gas);
         vec2_t rates = {{pitch, roll}};
         cm_att_set_rates(rates);
         cm_yaw_set_spd(yaw);
         break;
      }

      case MAN_RELAXED:
      {
         cm_u_acc_set(gas);
         vec2_t rates = {{pitch, roll}};
         cm_att_set_rates(rates);
         cm_yaw_set_spd(yaw);
         break;
      }

      case MAN_NOVICE:
      {
         cm_u_spd_set(gas);
         vec2_t speed = {{pitch, roll}};
         cm_att_set_gps_spd(speed);
         cm_yaw_set_spd(yaw);
         break;
      }
   }
}

