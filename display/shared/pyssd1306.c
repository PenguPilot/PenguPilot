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


#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

#include "i2c/i2c.h"
#include "ssd1306.h"


#define ASSERT_INIT() if (!initialized) return


static i2c_bus_t i2c_bus;
static i2c_dev_t i2c_dev;
static ssd1306_t ssd;
static bool initialized = false;


void init(char *_i2c_bus, int16_t w, int16_t h)
{
   assert(_i2c_bus);
   if (initialized)
      return;
   i2c_bus_open(&i2c_bus, _i2c_bus);
   i2c_dev_init(&i2c_dev, &i2c_bus, 0x3d);
   ssd1306_init(&ssd, &i2c_dev, w, h);
   initialized = true;
}


void blit(char *data)
{
   for (int x = 0; x < ssd.width; x++)
      for (int y = 0; y < ssd.height; y++)
         ssd1306_set_pixel(&ssd, x, y, data[x + y * ssd.width]);   
   ssd1306_update(&ssd);
}


void set_pixel(int16_t x, int16_t y, uint16_t color)
{
   ASSERT_INIT();
   ssd1306_set_pixel(&ssd, x, y, color);   
}


void invert(bool i)
{
   ASSERT_INIT();
   ssd1306_invert(&ssd, i);   
}


void clear(void)
{
   ASSERT_INIT();
   ssd1306_clear(&ssd);
}


void update(void)
{
   ASSERT_INIT();
   ssd1306_update(&ssd);
}

