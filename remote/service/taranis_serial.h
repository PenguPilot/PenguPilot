

#ifndef __TARANIS_SERIAL_H__
#define __TARANIS_SERIAL_H__


int taranis_serial_open(const char *device);

int taranis_serial_read(int fd);


#endif /* __TARANIS_SERIAL_H__ */

