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
  
 Adafruit 128x64 SSD1307 Linux I2C Driver Interface

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology
 Based on code written by Limor Fried/Ladyada.  

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "i2c/i2c.h"
#include <stdint.h>


typedef struct
{
   i2c_dev_t *i2c_dev;
   uint8_t *buf;
   int16_t width;
   int16_t height;
}
ssd1306_t;


/* initializes display */
void ssd1306_init(ssd1306_t *ssd, i2c_dev_t *i2c_dev);

/* inverts display */
void ssd1306_invert(ssd1306_t *ssd, uint8_t inv);

/* writes buffer contents to display */
void ssd1306_update(ssd1306_t *ssd);

/* clears buffer */
void ssd1306_clear(ssd1306_t *ssd);

/* sets pixel in buffer */
void ssd1306_set_pixel(ssd1306_t *ssd, int16_t x, int16_t y, uint16_t color);


#endif /* __SSD1306_H__ */

