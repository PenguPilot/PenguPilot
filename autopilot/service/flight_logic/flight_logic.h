

#ifndef __FLIGHT_LOGIC_H__
#define __FLIGHT_LOGIC_H__


#include <stdbool.h>
#include <stdint.h>
#include "../hardware/util/rc_channels.h"
#include "../util/math/vec2.h"


void flight_logic_init(void);


void flight_logic_run(uint16_t sensor_status, bool flying, float channels[MAX_CHANNELS], float yaw, vec2_t *ne_gps_pos, float u_baro_pos, float u_ultra_pos);


#endif /* __FLIGHT_LOGIC_H__ */

