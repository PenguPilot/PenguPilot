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
 
 Autopilot Telemetry Serial Interface

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#define pp_uart      Serial3



int ap_base_mode = 0;
int ap_custom_mode = 0;
int ap_system_status = 0;

int voltage = 0; // 1000 = 1V
int current = 0;    //  100 = 1A
int cells = 4;


int fix = 3;                  //   0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
int lat = 585522540;
int lon = 162344467;
int gps_alt = 1000;        // 1000 = 1m
int gps_hdop = 100;
int gps_speed = 46000 / 20;
int heading = 10;
int baro_alt = 20;
int vspd = 500;



void ap_init()
{
   pp_uart.begin(100000, SERIAL_8N2); /* should be 8E2 */
}


void ap_tx(char byte)
{
   pp_uart.write(byte);
}


int ap_rx(void)
{
   int status = 0;
   if (pp_uart.available()) 
   {
      static char buffer[1024];
      static int i = 0;
      char c = pp_uart.read();
      if (i < sizeof(buffer))
      {
         buffer[i++] = c;
      }
      if (c == '\n')
      {
         buffer[i] = '\0';
         if (sscanf(buffer, "H %d %d", &current, &voltage) == 2)
         {
            status = 1;
         }
         else if (sscanf(buffer, "G %d %d", &lat, &lon) == 2)
         {
            status = 1;
         }
         i = 0;
      }

   }
   return status;
}

