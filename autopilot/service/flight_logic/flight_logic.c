

#include "flight_logic.h"
#include "man_logic.h"
#include "../main_loop/control_mode.h"



static enum
{
   MODE_MANUAL,
   MODE_SAFE_AUTO,
   MODE_FULL_AUTO
}
flight_mode = MODE_MANUAL;



void flight_logic_init(void)
{
   switch (flight_mode)
   {
      case MODE_MANUAL:
         man_logic_init();
         break;

      case MODE_SAFE_AUTO:
         break;

      case MODE_FULL_AUTO:
         break;
   }
}


void flight_logic_run(uint16_t sensor_status, bool flying, float channels[MAX_CHANNELS], float yaw, vec2_t *ne_gps_pos, float u_baro_pos, float u_ultra_pos)
{
   switch (flight_mode)
   {
      case MODE_MANUAL:
         man_logic_run(sensor_status, flying, channels, yaw, ne_gps_pos, u_baro_pos, u_ultra_pos);
         break;

      case MODE_SAFE_AUTO:
         break;

      case MODE_FULL_AUTO:
         break;
   }
}

