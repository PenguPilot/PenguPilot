/*___________________________________________________
 |  _____                       _____ _ _       _    |
 | |  __ \                     |  __ (_) |     | |   |
 | | |__) |__ _ __   __ _ _   _| |__) || | ___ | |_  |
 | |  ___/ _ \ '_ \ / _` | | | |  ___/ | |/ _ \| __| |
 | | |  |  __/ | | | (_| | |_| | |   | | | (_) | |_  |
 | |_|   \___|_| |_|\__, |\__,_|_|   |_|_|\___/ \__| |
 |                   __/ |                           |
 |  GNU/Linux based |___/  Multi-Rotor UAV Autopilot |
 |___________________________________________________|
  
 Adafruit 128x64 SSD1307 Linux I2C Driver Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology
 Based on code written by Limor Fried/Ladyada.  

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "ssd1306.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>


/* internal declarations: */

#define SSD_COMMAND_MODE      0x00
#define SSD_DATA_MODE         0x40
#define SSD_INVERSE_DISPLAY   0xA7
#define SSD_DISPLAY_OFF       0xAE
#define SSD_DISPLAY_ON        0xAF
#define SSD_SET_CONTRAST      0x81
#define SSD_EXTERNAL_VCC      0x01
#define SSD_INTERNAL_VCC      0x02
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


static void ssd1306_cmd1(ssd1306_t *ssd, uint8_t c);
static void ssd1306_cmd2(ssd1306_t *ssd, uint8_t c0, uint8_t c1);
static void ssd1306_cmd3(ssd1306_t *ssd, uint8_t c0, uint8_t c1, uint8_t c2);


/* public functions: */


void ssd1306_init(ssd1306_t *ssd, i2c_dev_t *i2c_dev) 
{
   assert(ssd);
   assert(i2c_dev);
   
   ssd->i2c_dev = i2c_dev;
   ssd->width  = 128;
   ssd->height = 64;
   ssd->buf = (uint8_t *)malloc((ssd->width * ssd->height / 8)); 

   ssd1306_cmd1(ssd, SSD_DISPLAY_OFF);
   ssd1306_cmd2(ssd, SSD1306_SETDISPLAYCLOCKDIV, 0x80);
   ssd1306_cmd2(ssd, SSD1306_SETMULTIPLEX, 0x3F);
   ssd1306_cmd2(ssd, SSD1306_SETDISPLAYOFFSET, 0x00);
   ssd1306_cmd1(ssd, SSD1306_SETSTARTLINE);
   ssd1306_cmd2(ssd, SSD1306_CHARGEPUMP, 0x14); 
   ssd1306_cmd2(ssd, SSD1306_MEMORYMODE, 0x00);
   ssd1306_cmd1(ssd, SSD1306_SEGREMAP | 0x1);
   ssd1306_cmd1(ssd, SSD1306_COMSCANDEC);
   ssd1306_cmd2(ssd, SSD1306_SETCOMPINS, 0x12);
   ssd1306_cmd2(ssd, SSD_SET_CONTRAST, 0xFF);
   ssd1306_cmd2(ssd, SSD1306_SETPRECHARGE, 0xF1);
   ssd1306_cmd2(ssd, SSD1306_SETVCOMDETECT, 0x40);
   ssd1306_cmd1(ssd, SSD1306_DISPLAYALLON_RESUME);
   ssd1306_cmd1(ssd, SSD1306_Normal_Display);

   ssd1306_cmd3(ssd, 0x21, 0, 127); 
   ssd1306_cmd3(ssd, 0x22, 0, 7); 
   ssd1306_cmd1(ssd, SSD_DEACTIVATE_SCROLL);
   
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


void ssd1306_update(ssd1306_t *ssd)
{
   assert(ssd);
   ssd1306_cmd1(ssd, SSD1306_SETLOWCOLUMN | 0x0);
   ssd1306_cmd1(ssd, SSD1306_SETHIGHCOLUMN | 0x0);
   ssd1306_cmd1(ssd, SSD1306_SETSTARTLINE | 0x0);

   uint8_t *p = ssd->buf;
   uint8_t buf[17] ;
   buf[0] = SSD_DATA_MODE; 
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


void ssd1306_set_pixel(ssd1306_t *ssd, int16_t x, int16_t y, uint16_t color) 
{
   assert(ssd);
   if ((x < 0) || (x >= ssd->width) || (y < 0) || (y >= ssd->height))
      return;
   uint8_t *p = ssd->buf + (x + (y / 8) * ssd->width );
   if (color) 
      *p |= 1 << (y % 8);  
   else
      *p &= ~(1 << (y % 8)); 
}


/* internal functions: */


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

