
#ifndef __MAN_LOGIC_H__
#define __MAN_LOGIC_H__


#include <stdint.h>
#include "../hardware/util/rc_channels.h"
#include "../util/math/vec2.h"


void man_logic_init(void);


void man_logic_run(uint16_t sensor_status, float channels[MAX_CHANNELS], float yaw, vec2_t *ne_gps_pos, float u_baro_pos, float u_ultra_pos);


#endif /* __MAN_LOGIC_H__ */

