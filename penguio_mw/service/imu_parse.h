#ifndef __IMU_PARSE_H__
#define __IMU_PARSE_H__

#include <stdint.h>

#define IMU_SYNC_BYTE  0xAC

struct imu_sentence_s
{
	uint8_t sync;
	int16_t gyro[3];
	int16_t acc[3];
	int16_t mag[3];
	float alt;
	uint16_t checksum;
} __attribute__((packed));

extern int imu_parse_frame(struct imu_sentence_s *data, uint8_t c);

#endif /* __IMU_PARSE_H__ */
