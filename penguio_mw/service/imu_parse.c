#include <stdio.h>

#include "checksum.h"
#include "imu_parse.h"

static enum
{
	READ_SYNC,
	READ_GYRO_0,
	READ_GYRO_1,
	READ_ACC_0,
	READ_ACC_1,
	READ_MAG_0,
	READ_MAG_1,
	READ_ALT_0,
	READ_ALT_1,
	READ_ALT_2,
	READ_ALT_3,
	READ_CS0,
	READ_CS1
} state = READ_SYNC;

int imu_parse_frame(struct imu_sentence_s *data, uint8_t c)
{
	int ret = 0;
	static int cnt = 0;

	switch(state)
	{
		case READ_SYNC:
			if(c == (uint8_t)(IMU_SYNC_BYTE))
			{
				state = READ_GYRO_0;
				data->sync = c;
				cnt = 0;
			}
			break;

		/* Gyro */
		case READ_GYRO_0:
			*(uint16_t *)&(data->gyro[cnt]) = c;
			state = READ_GYRO_1;
			break;
		case READ_GYRO_1:
			*(uint16_t *)&(data->gyro[cnt]) |= (uint16_t)(c) << 8;
			if(++cnt < 3)
			{
				state = READ_GYRO_0;
			} else {
				state = READ_ACC_0;
				cnt = 0;
			}
			break;

		/* ACC */
		case READ_ACC_0:
			*(uint16_t *)&(data->acc[cnt]) = c;
			state = READ_ACC_1;
			break;
		case READ_ACC_1:
			*(uint16_t *)&(data->acc[cnt]) |= (uint16_t)(c) << 8;
			if(++cnt < 3)
			{
				state = READ_ACC_0;
			} else {
				state = READ_MAG_0;
				cnt = 0;
			}
			break;

		/* Mag */
		case READ_MAG_0:
			*(uint16_t *)&(data->mag[cnt]) = c;
			state = READ_MAG_1;
			break;
		case READ_MAG_1:
			*(uint16_t *)&(data->mag[cnt]) |= (uint16_t)(c) << 8;
			if(++cnt < 3)
			{
				state = READ_MAG_0;
			} else {
				state = READ_ALT_0;
				cnt = 0;
			}
			break;

		/* Alt */
		case READ_ALT_0:
			*(uint32_t *)&(data->alt) = c;
			state = READ_ALT_1;
			break;
		case READ_ALT_1:
			*(uint32_t *)&(data->alt) |= (uint32_t)(c) << 8;
			state = READ_ALT_2;
			break;
		case READ_ALT_2:
			*(uint32_t *)&(data->alt) |= (uint32_t)(c) << 16;
			state = READ_ALT_3;
			break;
		case READ_ALT_3:
			*(uint32_t *)&(data->alt) |= (uint32_t)(c) << 24;
			state = READ_CS0;
			break;

		/* cs */
		case READ_CS0:
			data->checksum = c;
			state = READ_CS1;
			break;
		case READ_CS1:
			data->checksum |= c << 8;
			if(data->checksum == checksum(
					(uint8_t *)(data),
					sizeof(struct imu_sentence_s)
					- sizeof(data->checksum)))
			{
				ret = sizeof(struct imu_sentence_s);
			}
			state = READ_SYNC;
			break;
	}

	return ret;
}

