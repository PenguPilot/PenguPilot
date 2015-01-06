#include "adns_3080.h"
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <errno.h>
#include <time.h>

// Spi options
static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 2000000;
static uint16_t delay = 0;

int adns3080_init(const char *device)
{
	int ret = 0;

	//Open Device
	int fd = open(device, O_RDWR); //Device öffnen
	if (fd == -1)
	{
		fprintf(stderr, "Error: can't open device");
		return -1;
	}

	//Set SPI-mode an read SPI-mode
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
	{
		fprintf(stderr, "Error: can't set spi mode");
		return -1;
	}
	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
	{
		fprintf(stderr, "Error: can't get spi mode");
		return -1;
	}

	//Set bits per word
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		fprintf(stderr, "Error: can't set bits per word");
		return -1;
	}


	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		fprintf(stderr, "Error: can't get bits per word");
		return -1;
	}

	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		fprintf(stderr, "Error: can't set max speed hz");
		return -1;
	}

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		fprintf(stderr, "Error: can't get max speed hz");
		return -1;
	}

	int i = 0;
	while(1)
	{
		uint8_t id = adns3080_spi_write(fd, 0x00, 0x00);

		if(id == 0x17)
			break;
		else
			fprintf(stderr, "Error: cannot dedect ADNS3080\n");
		i++;
		usleep(500);
	}

	printf("Info: Init successful (%i attempt(s))\n", i);

	return fd;
}

uint8_t adns3080_spi_write(int fd, uint8_t address, uint8_t value)
{
	uint8_t buf[2];
	buf[0] = address;
	buf[1] = value;

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 2,
	    .delay_usecs = 75,
	};

	int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if(ret < 1)
		fprintf(stderr, "Error: cant send spi message\n");

	return buf[1];
}

int adns3080_read_image(int fd, uint8_t *buf)
{
	adns3080_spi_write(fd, 0x93, 0x83);
	usleep(2500);

	uint8_t frame[1500];
	frame[0] = 0x40;
	struct spi_ioc_transfer tr = {
	    .tx_buf = (unsigned long)frame,
	    .rx_buf = (unsigned long)buf,
	    .len = 1500,
	    .delay_usecs = 50,
	};

	int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
	{
		int error = errno;
	    perror("adns3080_read_frame, can't send spi message");
	    fprintf(stderr, "ret: %d, errno: %d\n", ret, error);
	    return -1;
	}

	if((frame[1] & 0x40) != 0x40) // First pixel has bit 6 set
	{
//		fprintf(stderr, "Fist byte of frame not as expected: %X\n", frame[1]);
		return -1;
	}
   return 0;
}

