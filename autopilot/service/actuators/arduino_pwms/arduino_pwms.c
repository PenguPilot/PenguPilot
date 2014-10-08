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
  
 Arduino Serial ESC Bridge Driver Implementation

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau
 Copyright (C) 2014 Jan Roemisch, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include <string.h>

#include <util.h>
#include <serial.h>
#include <simple_thread.h>
#include <opcd_interface.h>
#include <pthread.h>

#include "pwm_common.h"
#include "pwm_build.h"


static serialport_t port;
static char *dev_path = NULL;
static int dev_speed = 0;


int arduino_pwms_init(void)
{
   ASSERT_ONCE();
   THROW_BEGIN();
   opcd_param_get("exynos_quad.arduino_serial.path", &dev_path);
   opcd_param_get("exynos_quad.arduino_serial.speed", &dev_speed);
   THROW_ON_ERR(serial_open(&port, dev_path, dev_speed, O_WRONLY));
   THROW_END();
}


int arduino_pwms_write(float *setpoints)
{
   uint8_t values[PWM_COUNT_MAX];
   uint8_t buffer[16];

   FOR_N(i, PWM_COUNT_MAX)
      values[i] = (uint8_t)(setpoints[i] * 120.0f + 127.0f);

   size_t len = pwm_build_frame(buffer, values, PWM_COUNT_MAX);
   int ret = serial_write(&port, (const char *)buffer, len);	
   return (ret < 0) ? -EAGAIN : 0;
}

