/******************************************************************
 This is the core graphics library for all our displays, providing
 basic graphics primitives (points, lines, circles, etc.). It needs
 to be paired with a hardware-specific library for each display
 device we carry (handling the lower-level functions).
 
 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source hardware
 by purchasing products from Adafruit!
 
 Written by Limor Fried/Ladyada for Adafruit Industries.
 BSD license, check license.txt for more information.
 All text above must be included in any redistribution.
 
02/18/2013 	Charles-Henri Hallard (http://hallard.me)
						Modified for compiling and use on Raspberry ArduiPi Board
						LCD size and connection are now passed as arguments on 
						the command line (no more #define on compilation needed)
						ArduiPi project documentation http://hallard.me/arduipi

 ******************************************************************/

#ifndef __PYSSD1306_H__
#define __PYSSD1306_H__


#include <stdint.h>
#include <stdbool.h>


void init(char *i2c_bus, int16_t w, int16_t h);

void blit(char *data);

void set_pixel(int16_t x, int16_t y, uint16_t color);

void invert(bool i);

void clear(void);

void update(void);


#endif /* __PYSSD1306_H__ */

