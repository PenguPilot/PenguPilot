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
 
 Teensy Taranis Firmware Program Entry Point

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */




#include "s_bus.h"
#include "led.h"


void setup()
{
   led_init();
   s_port_init();
   s_bus_init(); 
   ap_init();
}


void loop()
{
   /* update led flashing signal: */
   static uint8_t flags = 0;
   flags = led_update(flags);

   /* provide telemetry to remote control via s-port RX/TX (1-wire) */
   if (s_port_rx_tx())
      flags |= S_PORT_ACTIVITY;

   /* copy from s-bus TX to AP rx */
   if (s_bus_copy(ap_tx))
      flags |= S_BUS_ACTIVITY;

   /* read data from autopilot via AP RX into global variables: */
   if (ap_rx())
      flags |= AP_ACTIVITY;
}
