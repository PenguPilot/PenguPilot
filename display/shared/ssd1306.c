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
All text above, and the splash screen below must be included in any redistribution

02/18/2013  Charles-Henri Hallard (http://hallard.me)
                  Modified for compiling and use on Raspberry ArduiPi Board
                  LCD size and connection are now passed as arguments on 
                  the command line (no more #define on compilation needed)
                  ArduiPi project documentation http://hallard.me/arduipi
07/01/2013  Charles-Henri Hallard 
                  Reduced code size removed the Adafruit Logo (sorry guys)
                  Buffer for OLED is now dynamic to LCD size
                  Added support of Seeed OLED 64x64 Display

*********************************************************************/


#include "ssd1306.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>


#define SSD_COMMAND_MODE      0x00
#define SSD_DATA_MODE         0x40
#define SSD_INVERSE_DISPLAY   0xA7
#define SSD_DISPLAY_OFF       0xAE
#define SSD_DISPLAY_ON        0xAF
#define SSD_SET_CONTRAST      0x81
#define SSD_EXTERNAL_VCC      0x01
#define SSD_INTERNAL_VCC      0x02
#define SSD_ACTIVATE_SCROLL   0x2F
#define SSD_DEACTIVATE_SCROLL 0x2E

#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON        0xA5
#define SSD1306_Normal_Display      0xA6

#define SSD1306_SETDISPLAYOFFSET    0xD3
#define SSD1306_SETCOMPINS          0xDA
#define SSD1306_SETVCOMDETECT       0xDB
#define SSD1306_SETDISPLAYCLOCKDIV  0xD5
#define SSD1306_SETPRECHARGE        0xD9
#define SSD1306_SETMULTIPLEX        0xA8
#define SSD1306_SETLOWCOLUMN        0x00
#define SSD1306_SETHIGHCOLUMN       0x10
#define SSD1306_SETSTARTLINE        0x40
#define SSD1306_MEMORYMODE          0x20
#define SSD1306_COMSCANINC          0xC0
#define SSD1306_COMSCANDEC          0xC8
#define SSD1306_SEGREMAP            0xA0
#define SSD1306_CHARGEPUMP          0x8D

#define SSD1306_SET_VERTICAL_SCROLL_AREA             0xA3
#define SSD1306_RIGHT_HORIZONTAL_SCROLL              0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL               0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  0x2A


static void ssd1306_cmd1(ssd1306_t *ssd, uint8_t c);
static void ssd1306_cmd2(ssd1306_t *ssd, uint8_t c0, uint8_t c1);
static void ssd1306_cmd3(ssd1306_t *ssd, uint8_t c0, uint8_t c1, uint8_t c2);


void ssd1306_set_pixel(ssd1306_t *ssd, int16_t x, int16_t y, uint16_t color) 
{
   assert(ssd);
   uint8_t *p = ssd->buf;
   
   if ((x < 0) || (x >= ssd->width) || (y < 0) || (y >= ssd->height))
      return;

   // Get where to do the change in the bufer
   p = ssd->buf + (x + (y / 8) * ssd->width );
   #define  _BV(bit)   (1 << (bit))
   // x is which column
   if (color) 
      *p |= _BV((y % 8));  
   else
      *p &= ~_BV((y % 8)); 
}


void ssd1306_init(ssd1306_t *ssd, i2c_dev_t *i2c_dev, int16_t w, int16_t h) 
{
   assert(ssd);
   assert(i2c_dev);
   uint8_t multiplex;
   uint8_t chargepump;
   uint8_t compins;
   uint8_t contrast;
   uint8_t precharge;
   
   ssd->i2c_dev = i2c_dev;
   uint8_t vcc_type;
   ssd->width  = w;
   ssd->height = h;
   vcc_type = SSD_INTERNAL_VCC;
   ssd->buf = (uint8_t *)malloc((ssd->width * ssd->height / 8)); 

   if (ssd->height == 32)
   {
      multiplex = 0x1F;
      compins  = 0x02;
      contrast = 0x8F;
   }
   else
   {
      multiplex = 0x3F;
      compins  = 0x12;
      contrast = (vcc_type == SSD_EXTERNAL_VCC?0x9F:0xCF);
   }
   
   if (vcc_type == SSD_EXTERNAL_VCC)
   {
      chargepump = 0x10; 
      precharge  = 0x22;
   }
   else
   {
      chargepump = 0x14; 
      precharge  = 0xF1;
   }
   
   ssd1306_cmd1(ssd, SSD_DISPLAY_OFF);
   ssd1306_cmd2(ssd, SSD1306_SETDISPLAYCLOCKDIV, 0x80);
   ssd1306_cmd2(ssd, SSD1306_SETMULTIPLEX, multiplex);
   ssd1306_cmd2(ssd, SSD1306_SETDISPLAYOFFSET, 0x00);
   ssd1306_cmd1(ssd, SSD1306_SETSTARTLINE);
   ssd1306_cmd2(ssd, SSD1306_CHARGEPUMP, chargepump); 
   ssd1306_cmd2(ssd, SSD1306_MEMORYMODE, 0x00);
   ssd1306_cmd1(ssd, SSD1306_SEGREMAP | 0x1);
   ssd1306_cmd1(ssd, SSD1306_COMSCANDEC);
   ssd1306_cmd2(ssd, SSD1306_SETCOMPINS, compins);
   ssd1306_cmd2(ssd, SSD_SET_CONTRAST, contrast);
   ssd1306_cmd2(ssd, SSD1306_SETPRECHARGE, precharge);
   ssd1306_cmd2(ssd, SSD1306_SETVCOMDETECT, 0x40);
   ssd1306_cmd1(ssd, SSD1306_DISPLAYALLON_RESUME);
   ssd1306_cmd1(ssd, SSD1306_Normal_Display);

   // Reset to default value in case of 
   // no reset pin available on OLED
   ssd1306_cmd3(ssd, 0x21, 0, 127); 
   ssd1306_cmd3(ssd, 0x22, 0, 7); 
   ssd1306_stop_scroll(ssd);
   
   ssd1306_clear(ssd);
   ssd1306_update(ssd);
   ssd1306_cmd1(ssd, SSD_DISPLAY_ON);
}


void ssd1306_invert(ssd1306_t *ssd, uint8_t inv)
{
   assert(ssd);
   if (inv) 
      ssd1306_cmd1(ssd, SSD_INVERSE_DISPLAY);
   else 
      ssd1306_cmd1(ssd, SSD1306_Normal_Display);
}


void ssd1306_start_scroll_right(ssd1306_t *ssd, uint8_t start, uint8_t stop)
{
   assert(ssd);
   ssd1306_cmd1(ssd, SSD1306_RIGHT_HORIZONTAL_SCROLL);
   ssd1306_cmd1(ssd, 0X00);
   ssd1306_cmd1(ssd, start);
   ssd1306_cmd1(ssd, 0X00);
   ssd1306_cmd1(ssd, stop);
   ssd1306_cmd1(ssd, 0X01);
   ssd1306_cmd1(ssd, 0XFF);
   ssd1306_cmd1(ssd, SSD_ACTIVATE_SCROLL);
}


void ssd1306_start_scroll_left(ssd1306_t *ssd, uint8_t start, uint8_t stop)
{
   assert(ssd);
   ssd1306_cmd1(ssd, SSD1306_LEFT_HORIZONTAL_SCROLL);
   ssd1306_cmd1(ssd, 0X00);
   ssd1306_cmd1(ssd, start);
   ssd1306_cmd1(ssd, 0X00);
   ssd1306_cmd1(ssd, stop);
   ssd1306_cmd1(ssd, 0X01);
   ssd1306_cmd1(ssd, 0XFF);
   ssd1306_cmd1(ssd, SSD_ACTIVATE_SCROLL);
}


void ssd1306_start_scroll_diag_right(ssd1306_t *ssd, uint8_t start, uint8_t stop)
{
   assert(ssd);
   ssd1306_cmd1(ssd, SSD1306_SET_VERTICAL_SCROLL_AREA); 
   ssd1306_cmd1(ssd, 0X00);
   ssd1306_cmd1(ssd, ssd->height);
   ssd1306_cmd1(ssd, SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
   ssd1306_cmd1(ssd, 0X00);
   ssd1306_cmd1(ssd, start);
   ssd1306_cmd1(ssd, 0X00);
   ssd1306_cmd1(ssd, stop);
   ssd1306_cmd1(ssd, 0X01);
   ssd1306_cmd1(ssd, SSD_ACTIVATE_SCROLL);
}


void ssd1306_start_scroll_diag_left(ssd1306_t *ssd, uint8_t start, uint8_t stop)
{
   assert(ssd);
   ssd1306_cmd1(ssd, SSD1306_SET_VERTICAL_SCROLL_AREA); 
   ssd1306_cmd1(ssd, 0X00);
   ssd1306_cmd1(ssd, ssd->height);
   ssd1306_cmd1(ssd, SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
   ssd1306_cmd1(ssd, 0X00);
   ssd1306_cmd1(ssd, start);
   ssd1306_cmd1(ssd, 0X00);
   ssd1306_cmd1(ssd, stop);
   ssd1306_cmd1(ssd, 0X01);
   ssd1306_cmd1(ssd, SSD_ACTIVATE_SCROLL);
}


void ssd1306_stop_scroll(ssd1306_t *ssd)
{
   assert(ssd);
   ssd1306_cmd1(ssd, SSD_DEACTIVATE_SCROLL);
}


void ssd1306_update(ssd1306_t *ssd)
{
   assert(ssd);
   ssd1306_cmd1(ssd, SSD1306_SETLOWCOLUMN | 0x0); // low col = 0
   ssd1306_cmd1(ssd, SSD1306_SETHIGHCOLUMN | 0x0); // hi col = 0
   ssd1306_cmd1(ssd, SSD1306_SETSTARTLINE | 0x0); // line #0

   uint8_t *p = ssd->buf;
   uint8_t buf[17] ;
   // Setup D/C to switch to data mode
   buf[0] = SSD_DATA_MODE; 
   // loop trough all OLED bufer and 
   // send a bunch of 16 data byte in one xmission
   for (uint16_t i = 0; i < (ssd->width * ssd->height / 8); i += 16) 
   {
      for (uint8_t x = 1; x <= 16; x++) 
         buf[x] = *p++;
      i2c_xfer(ssd->i2c_dev, sizeof(buf), buf, 0, NULL);
   }
}


void ssd1306_clear(ssd1306_t *ssd)
{
   assert(ssd);
   memset(ssd->buf, 0, (ssd->width * ssd->height / 8));
}


static void ssd1306_cmd1(ssd1306_t *ssd, uint8_t c)
{
   assert(ssd);
   uint8_t buf[2] ;
   buf[0] = SSD_COMMAND_MODE ; 
   buf[1] = c;
   i2c_xfer(ssd->i2c_dev, sizeof(buf), buf, 0, NULL);
}


static void ssd1306_cmd2(ssd1306_t *ssd, uint8_t c0, uint8_t c1)
{
   assert(ssd);
   uint8_t buf[3];
   buf[0] = SSD_COMMAND_MODE ;
   buf[1] = c0;
   buf[2] = c1;
   i2c_xfer(ssd->i2c_dev, sizeof(buf), buf, 0, NULL);
}


static void ssd1306_cmd3(ssd1306_t *ssd, uint8_t c0, uint8_t c1, uint8_t c2)
{
   assert(ssd);
   uint8_t buf[4] ;
   buf[0] = SSD_COMMAND_MODE; 
   buf[1] = c0;
   buf[2] = c1;
   buf[3] = c2;
   i2c_xfer(ssd->i2c_dev, sizeof(buf), buf, 0, NULL);
}

