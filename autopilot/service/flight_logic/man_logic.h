
#ifndef __MAN_LOGIC_H__
#define __MAN_LOGIC_H__


#include <stdint.h>
#include "../hardware/util/rc_channels.h"


void man_logic_init(void);


void man_logic_run(uint16_t sensor_status, float channels[MAX_CHANNELS]);


#endif /* __MAN_LOGIC_H__ */

