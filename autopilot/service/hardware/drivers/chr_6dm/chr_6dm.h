/*
 * chr-6dm.h
 *
 *  Created on: 30.06.2010
 *      Author: tobi
 */

#ifndef __CHR6DM_H__
#define __CHR6DM_H__


typedef struct
{
   /* euler angles: */
   float pitch; /* -PI .. PI, 0 is horizontal */
   float roll; /* -PI .. PI, 0 is horizontal */
   float yaw; /* -PI .. PI, 0 is north */

   /* angular speeds: */
   float pitch_rate; /* in rad / s */
   float roll_rate; /* in rad / s */
   float yaw_rate;  /* in rad / s */

   /* acc acceleration: */
   float acc_pitch; /* in m / (s ^ 2) */
   float acc_roll; /* in m / (s ^ 2) */
   float acc_yaw;  /* in rad / (s ^ 2) */
}
ahrs_data_t;


int chr6dm_init(void);

int chr6dm_read(ahrs_data_t *data);

void chr6dm_wait_for_data(void);


#endif /* __CHR6DM_H__ */

