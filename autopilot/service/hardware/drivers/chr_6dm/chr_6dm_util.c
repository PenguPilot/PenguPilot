
#include <math.h>
#include "chr_6dm_util.h"


static const float CHR6DM_SCALE_TABLE[CHR6DM_N_CHANNELS / CHR6DM_DIMENSIONS] =
{
   0.106812,  /* acc  x / y / z */
   0.01812,   /* gyro x / y / z */
   0.061035,  /* mag  x / y / z */
   0.0137329 * M_PI / 180.0, /* yaw / pitch / roll angle rate */
   0.0109863  /* yaw / pitch / roll angle */
};


static const char *CHR6DM_CHANNEL_NAMES[CHR6DM_N_CHANNELS] =
{
   "acc_z",
   "acc_y",
   "acc_x",
   "gyro_z",
   "gyro_y",
   "gyro_x",
   "mag_z",
   "mag_y",
   "mag_x",
   "roll_rate",
   "pitch_rate",
   "yaw_rate",
   "roll_angle",
   "pitch_angle",
   "yaw_angle"
};



float chr6dm_scale_table_entry(chr6dm_channel_t channel)
{
   return CHR6DM_SCALE_TABLE[channel / CHR6DM_DIMENSIONS];
}


const char *chr6dm_channel_name(chr6dm_channel_t channel)
{
   return CHR6DM_CHANNEL_NAMES[channel];
}

