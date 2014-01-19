

#ifndef __BLACKBOX_H__
#define __BLACKBOX_H__


#include "../platform/platform.h"


void blackbox_init(void);


void blackbox_record(float dt,
               marg_data_t *marg_data,
               gps_data_t *gps_data,
               float ultra,
               float baro,
               float voltage,
               float channels[MAX_CHANNELS],
               uint16_t sensor_status);

#endif /* __BLACKBOX_H__ */

