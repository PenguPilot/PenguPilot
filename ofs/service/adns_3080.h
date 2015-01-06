#ifndef ADNS3080_H_
#define ADNS3080_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

int adns3080_init(const char *device);
uint8_t adns3080_spi_write(int fd, uint8_t address, uint8_t value);
int adns3080_read_image(int fd, uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif
