/*********************************************************************
This is a library for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

These displays use SPI to communicate, 4 or 5 pins are required to  
interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution

02/18/2013 	Charles-Henri Hallard (http://hallard.me)
						Modified for compiling and use on Raspberry ArduiPi Board
						LCD size and connection are now passed as arguments on 
						the command line (no more #define on compilation needed)
						ArduiPi project documentation http://hallard.me/arduipi
 
*********************************************************************/


#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "i2c/i2c.h"
#include <stdint.h>


typedef struct
{
   uint8_t *buf;
   i2c_dev_t *i2c_dev;
   int16_t width;
   int16_t height;
}
ssd1306_t;


void ssd1306_init(ssd1306_t *ssd, i2c_dev_t *i2c_dev, int16_t w, int16_t h);

void ssd1306_clear(ssd1306_t *ssd);
void ssd1306_invert(ssd1306_t *ssd, uint8_t inv);
void ssd1306_update(ssd1306_t *ssd);

void ssd1306_start_scroll_right(ssd1306_t *ssd, uint8_t start, uint8_t stop);
void ssd1306_start_scroll_left(ssd1306_t *ssd, uint8_t start, uint8_t stop);

void ssd1306_start_scroll_diag_right(ssd1306_t *ssd, uint8_t start, uint8_t stop);
void ssd1306_start_scroll_diag_left(ssd1306_t *ssd, uint8_t start, uint8_t stop);
void ssd1306_stop_scroll(ssd1306_t *ssd);

void ssd1306_set_pixel(ssd1306_t *ssd, int16_t x, int16_t y, uint16_t color);


#endif /* __SSD1306_H__ */

