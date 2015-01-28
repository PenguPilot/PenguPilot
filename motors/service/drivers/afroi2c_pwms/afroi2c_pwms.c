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
  
 AfroI2C I2C-PWM Converter Implementation

 Copyright (C) 2014 Martin Turetschek, Integrated Communication Systems Group, TU Ilmenau
 Copyright (C) 2014 Kevin Ernst, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <util.h>

#include "afroi2c_pwms.h"


#define AFROI2C_PWMS_MAX_ESCS 8
#define AFROI2C_PWMS_ADDRESS	0x29
#define AFROI2C_PWMS_MIN 0x00
#define AFROI2C_PWMS_MAX 0xFF


static size_t _n_pwms = 0;
static i2c_dev_t device;
static uint8_t *_pwms_map = NULL;


int afroi2c_pwms_init(i2c_bus_t *bus, uint8_t *pwms_map, size_t n_pwms)
{
   ASSERT_ONCE();
   ASSERT_TRUE(n_pwms > 0 && n_pwms <= AFROI2C_PWMS_MAX_ESCS);
   _n_pwms = n_pwms;
   _pwms_map = pwms_map;
   i2c_dev_init(&device, bus, AFROI2C_PWMS_ADDRESS);
   return 0;
}


int afroi2c_pwms_write(float *setpoints)
{
    uint8_t data[AFROI2C_PWMS_MAX_ESCS];    
    THROW_BEGIN();
    FOR_N(i, _n_pwms)
    {
        uint16_t val = setpoints[i] * AFROI2C_PWMS_MAX;
        if (val > AFROI2C_PWMS_MAX)
           val = AFROI2C_PWMS_MAX;
        data[_pwms_map[i]] = (uint8_t)val;
    }
    THROW_ON_ERR(i2c_xfer(&device, _n_pwms, data, 0, NULL));
    THROW_END();
}

