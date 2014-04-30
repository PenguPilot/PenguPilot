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

 Copyright (C) 2014 Martin Turetschek, Ilmenau University of Technology
 Copyright (C) 2014 Kevin Ernst, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <util.h>
#include <stdint.h>

#include "afroi2c_escs.h"


#define AFROI2C_MAX_ESCS 8
#define AFROI2C_ADDRESS	0x29
#define AFROI2C_MIN 0x00
#define AFROI2C_MAX 0xFF


static size_t _n_escs;
static i2c_dev_t device;


void afroi2c_init(i2c_bus_t *bus, size_t n_escs)
{
    ASSERT_ONCE();
    ASSERT_TRUE(n_escs > 0 && n_escs <= AFROI2C_MAX_ESCS);
    _n_escs = n_escs;
    i2c_dev_init(&device, bus, AFROI2C_ADDRESS);
}


int afroi2c_write(float *setpoints)
{
    uint8_t data[AFROI2C_MAX_ESCS];    
    THROW_BEGIN();
    FOR_N(i, _n_escs)
    {
        uint16_t val = setpoints[i] * AFROI2C_MAX;
        if (val > AFROI2C_MAX)
           val = AFROI2C_MAX;
        data[i] = (uint8_t)val;
    }
    THROW_ON_ERR(i2c_xfer(&device, _n_escs, data, 0, NULL));
    THROW_END();
}

