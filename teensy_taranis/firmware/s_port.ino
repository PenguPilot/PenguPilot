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
 
 S.Port Implementation

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau
 Copyright (C) 2014 chsw and others, based on: https://github.com/chsw/MavLink_FrSkySPort

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include "bits.h"

#define SENSOR_ID_VARIO 0x00 // 0
#define SENSOR_ID_FLVSS 0xA1 // 1
#define SENSOR_ID_FAS 0x22 // 2
#define SENSOR_ID_GPS 0x83 // 3
//#define SENSOR_ID_RPM 0xE4 // 4
//#define SENSOR_ID_SP2UH 0x45 // 5
//#define SENSOR_ID_SP2UR 0xC6 // 6

// We'll use some undefined SENSOR_ID's so expect things to break if OpenTX changes
// Frsky-specific
#define START_STOP 0x7e
#define DATA_FRAME 0x10

//Frsky DATA ID's
#define FR_ID_ALTITUDE 0x0100 //ALT_FIRST_ID
#define FR_ID_VARIO 0x0110 //VARIO_FIRST_ID
#define FR_ID_VFAS 0x0210 //VFAS_FIRST_ID
#define FR_ID_CURRENT 0x0200 //CURR_FIRST_ID
#define FR_ID_CELLS 0x0300 //CELLS_FIRST_ID
#define FR_ID_CELLS_LAST 0x030F //CELLS_LAST_ID
#define FR_ID_T1 0x0400 //T1_FIRST_ID
#define FR_ID_T2 0x0410 //T2_FIRST_ID
#define FR_ID_RPM 0x0500 //RPM_FIRST_ID
#define FR_ID_FUEL 0x0600 //FUEL_FIRST_ID
#define FR_ID_ACCX 0x0700 //ACCX_FIRST_ID
#define FR_ID_ACCY 0x0710 //ACCY_FIRST_ID
#define FR_ID_ACCZ 0x0720 //ACCZ_FIRST_ID
#define FR_ID_LATLONG 0x0800 //GPS_LONG_LATI_FIRST_ID
#define FR_ID_GPS_ALT 0x0820 //GPS_ALT_FIRST_ID
#define FR_ID_SPEED 0x0830 //GPS_SPEED_FIRST_ID
#define FR_ID_GPS_COURSE 0x0840 //GPS_COURS_FIRST_ID
#define FR_ID_GPS_TIME_DATE 0x0850 //GPS_TIME_DATE_FIRST_ID
#define FR_ID_A3_FIRST 0x0900 //A3_FIRST_ID
#define FR_ID_A4_FIRST 0x0910 //A4_FIRST_ID
#define FR_ID_AIR_SPEED_FIRST 0x0A00 //AIR_SPEED_FIRST_ID
#define FR_ID_RSSI 0xF101 // used by the radio system
#define FR_ID_ADC1 0xF102 //ADC1_ID
#define FR_ID_ADC2 0xF103 //ADC2_ID
#define FR_ID_BATT 0xF104 // used by the radio system
#define FR_ID_SWR 0xF105 // used by the radio system


#define s_port_uart      Serial1
#define s_port_uart_c1   UART0_C1
#define s_port_uart_c3   UART0_C3
#define s_port_uart_s2   UART0_S2


boolean waitingForSensorId = false;
uint8_t cell_count = 0;
uint8_t latlong_flag = 0;
uint32_t latlong = 0;
uint8_t nextFLVSS = 0;
uint8_t nextFAS = 0;
uint8_t nextVARIO = 0;
uint8_t nextGPS = 0;
uint8_t nextDefault = 0;


void s_port_init(void) 
{
   s_port_uart.begin(57600);
   s_port_uart_c3 = UART_INVERT; // TX
   s_port_uart_s2 = UART_INVERT; // RX
   s_port_uart_c1 = UART_SINGLE_WIRE;
}


static uint16_t crc; // used for crc calc of frsky-packet


static void s_port_send_byte(uint8_t byte)
{
   s_port_uart.write(byte);
   crc += byte;     //0-1FF
   crc += crc >> 8; //0-100
   crc &= 0x00ff;
   crc += crc >> 8; //0-0FF
   crc &= 0x00ff;
}


static void s_port_send_frame(uint16_t id, uint32_t value)
{
   s_port_uart_c3 |= 32; // Transmit direction, to S.Port
   s_port_send_byte(DATA_FRAME);
   uint8_t *bytes = (uint8_t*)&id;
   s_port_send_byte(bytes[0]);
   s_port_send_byte(bytes[1]);
   bytes = (uint8_t*)&value;
   s_port_send_byte(bytes[0]);
   s_port_send_byte(bytes[1]);
   s_port_send_byte(bytes[2]);
   s_port_send_byte(bytes[3]);
   s_port_uart.write(0xFF - crc);
   s_port_uart.flush();
   s_port_uart_c3 ^= 32; // Transmit direction, from S.Port
   crc = 0;
}


void handle_request(uint8_t sensorId)
{
   uint32_t temp = 0;
   uint8_t offset;

   switch (sensorId)
   {
      case SENSOR_ID_FLVSS:
      {
         switch(nextFLVSS)
         {
            case 0:
               if (cell_count > 0) 
               {
                  // First 2 cells
                  offset = 0x00 | ((cell_count & 0xF)<<4);
                  temp=((voltage/(cell_count * 2)) & 0xFFF);
                  s_port_send_frame(FR_ID_CELLS,(temp << 20) | (temp << 8) | offset);  // Battery cell 0,1
               }
               break;
      
            case 1:    
               // Optional 3 and 4 Cells
               if (cell_count > 2) 
               {
                  offset = 0x02 | ((cell_count & 0xF)<<4);
                  temp = ((voltage/(cell_count * 2)) & 0xFFF);
                  s_port_send_frame(FR_ID_CELLS,(temp << 20) | (temp << 8) | offset);  // Battery cell 2,3
               }
               break;

            case 2:    // Optional 5 and 6 Cells
            if (cell_count > 4) 
            {
               offset = 0x04 | ((cell_count & 0xF)<<4);
               temp = ((voltage/(cell_count * 2)) & 0xFFF);
               s_port_send_frame(FR_ID_CELLS,(temp << 20) | (temp << 8) | offset);  // Battery cell 2,3
            }
            break;     
         }
         nextFLVSS++;
         if(nextFLVSS>2)
            nextFLVSS=0;
      }
      break;

      case SENSOR_ID_VARIO:
         switch(nextVARIO)
         {
            case 0:
               s_port_send_frame(FR_ID_VARIO, vspd);       // 100 = 1m/s
               break;
            case 1: 
               s_port_send_frame(FR_ID_ALTITUDE, baro_alt);   // from barometer, 100 = 1m
               break;
         }
         if(++nextVARIO > 1)
            nextVARIO = 0;
         break;
  
      case SENSOR_ID_FAS:
         switch(nextFAS)
         {
            case 0:
               s_port_send_frame(FR_ID_VFAS, voltage); // Sends voltage as a VFAS value
               break;
            case 1:
               s_port_send_frame(FR_ID_CURRENT, current / 10);
         }
         if (++nextFAS > 1)
            nextFAS = 0;
         break;
  
      case SENSOR_ID_GPS:
      {
         switch (nextGPS)
         {
            case 0:        // Sends the ap_lon value, setting bit 31 high
               if (fix >= 2) 
               {
                  if (lon < 0)
                     latlong=((abs(lon)/100)*6)  | 0xC0000000;
                  else
                     latlong=((abs(lon)/100)*6)  | 0x80000000;
                  s_port_send_frame(FR_ID_LATLONG,latlong);
               }
               break;

            case 1:        // Sends the ap_lat value, setting bit 31 low  
               if (fix >= 2)
               {
                  if (lat < 0)
                     latlong=((abs(lat)/100)*6) | 0x40000000;
                  else
                     latlong=((abs(lat)/100)*6);
                  s_port_send_frame(FR_ID_LATLONG,latlong);
               }
               break;

            case 2:
               if (fix == 3)
                  s_port_send_frame(FR_ID_GPS_ALT, gps_alt / 10);   // from GPS,  100=1m
               break;
      
            case 3:
               if (fix >= 2)
                  s_port_send_frame(FR_ID_SPEED, gps_speed *20 );  // from GPS converted to km/h
               break;

            case 4:
               s_port_send_frame(FR_ID_GPS_COURSE, heading * 100);   // 10000 = 100 deg
         }
         if (++nextGPS > 4)
            nextGPS = 0;
      }
      break;

      case 0x45:
      case 0xC6:
         switch(nextDefault)
         {
            case 0:        // Note: We are using A2 - previously reported analog voltage when connected to Teensy - as Hdop
               s_port_send_frame(FR_ID_ADC2, gps_hdop);
               break;       
            case 1:
               //s_port_send_frame(FR_ID_ACCX, fetchAccX());    
               break;
            case 2:
               //s_port_send_frame(FR_ID_ACCY, fetchAccY()); 
               break; 
            case 3:
               //s_port_send_frame(FR_ID_ACCZ, fetchAccZ()); 
               break; 
            case 4:
               //s_port_send_frame(FR_ID_T1, gps_status); 
               break; 
            case 5:
               break;
            case 6:
               break;      
         }
         if (++nextDefault > 6)
            nextDefault = 0;
   }
}



int s_port_rx_tx(void)
{
   int status = 0;
   uint8_t data = 0;
   while (s_port_uart.available())
   {
      data = s_port_uart.read();
      if (data == START_STOP)
      {
         waitingForSensorId = true;
         continue;
      }
      if(!waitingForSensorId)
         continue;
      handle_request(data);
      waitingForSensorId = false;
      status = 1;
   }
   return status;
}


