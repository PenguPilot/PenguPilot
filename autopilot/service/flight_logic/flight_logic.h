

#ifndef __FLIGHT_LOGIC_H__
#define __FLIGHT_LOGIC_H__


#include <stdint.h>
#include "../hardware/util/rc_channels.h"


void flight_logic_init(void);


void flight_logic_run(uint16_t sensor_status, float channels[MAX_CHANNELS], float yaw);


#endif /* __FLIGHT_LOGIC_H__ */

