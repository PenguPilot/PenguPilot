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
  
 DSL Driver Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "rc_dsl.h"

#include <string.h>


#define CHANNEL_SCALE(x) (((float)x) / 2000.0f)



void rc_dsl_init(rc_dsl_t *dsl)
{
   memset(dsl, 0, sizeof(rc_dsl_t));
}


/*
 * new rc signal received
 *
 * this function is called, when a new servo signal is properly received.
 * parameters: servo  - servo number (0-9)
 *             signal - servo signal between 7373 (1ms) and 14745 (2ms)
 */
static void rc_dsl_new_dsl_rcsignal(rc_dsl_t *dsl, uint8_t servo, int16_t signal)
{
   /* scale signal: */
   signal -= 11059; /* shift neutral position to 0 */
   signal /= 6; /* scale 10bit [-512..512] */
   signal *= 4; /* scale 12bit [-2048..2048] */

   /* limit signal: */
   if (signal > 2048)
   {
      signal = 2048;
   }
   if (signal < -2048)
   {
      signal = -2048;
   }

   /* store signal: */
   dsl->channels[servo] = CHANNEL_SCALE(signal);
}


/*
 * parse a real DSL packet
 *
 * this function is called within rc_dsl_parse_data(), when a complete
 * data paket with matching checksum has been received.
 */
static int rc_dsl_parse_incoming_dsl_paket(rc_dsl_t *dsl)
{
   if (dsl->raw_data[0] == 0x1F)
   {
      /* process status header: */
      dsl->status_packets++;
      dsl->allocation = dsl->raw_data[0 + 1];
      dsl->channel = dsl->raw_data[1 + 1];
      dsl->RSSI = RSSI_SCALE(dsl->raw_data[2 + 1]);
      dsl->battery = dsl->raw_data[3 + 1];
      return 0;
   }
   else if ((dsl->raw_data[0] & 0xF0) == 0x10)
   {
      /* process signal header: */
      uint8_t i;
      dsl->signal_packets++;

      /* last 4 bits of the header indicates servo pair: */
      i = dsl->raw_data[0] & 0x0F;

      if (i < 10)
      {
         typedef union
         {
            uint16_t pos[2];
            uint8_t dat[4];
         }
         rc_dsl_servos_t;

         rc_dsl_servos_t servos;

         /* convert byte array to two uint16: */
         servos.dat[1] = dsl->raw_data[1];
         servos.dat[0] = dsl->raw_data[2];
         servos.dat[3] = dsl->raw_data[3];
         servos.dat[2] = dsl->raw_data[4];

         /* store new servo data: */
         rc_dsl_new_dsl_rcsignal(dsl, i, (int16_t) servos.pos[0]);
         rc_dsl_new_dsl_rcsignal(dsl, i + 1, (int16_t) servos.pos[1]);
      }
      return 0;
   }

   /* unknown packet type */
   dsl->packet_unknown++;
   return -1;
}


/*
 * parses a single DSL data stream byte
 * returns:
 *    0 if char was processed but no new frame was assembled
 *    1 if a valid frame was assembled
 *   -1 if the frame was invalid
 */
int rc_dsl_parse_dsl_data(rc_dsl_t *dsl, uint8_t b)
{
   int status = 0;

   /* check for sync condition */
   if ((b == 0xFF) && (dsl->last_byte == 0xFF))
   {
      dsl->data_counter = 0;
      dsl->check_sum = 0;
      goto out;
   }

   /* first byte is cmd */
   if (dsl->data_counter == 0)
   {
      if (b == 0x1F)
      {
         dsl->packet_len = 5;
      }
      else if ((b & 0xF0) == 0x10)
      {
         dsl->packet_len = 4;
      }
      else if (b == 0xE3)
      {
         dsl->packet_len = 48;
      }
   }

   /* last byte is checksum */
   if ((b != 0xFF) && (dsl->data_counter > dsl->packet_len))
   {
      /* calculate checksum */
      dsl->check_sum = ~(dsl->check_sum);
      if (dsl->check_sum == 0xFF)
      {
         dsl->check_sum = 0xFE;
      }

      if (b == dsl->check_sum)
      {
         status = rc_dsl_parse_incoming_dsl_paket(dsl);
         if (status == 0)
         {
            status = 1;
         }
      }
      else
      {
         status = -1;
         dsl->packet_invalid++;
      }

      /* prepare for a new data paket */
      dsl->data_counter = 0;
      dsl->check_sum = 0;
   }
   else
   {
      /* new byte within a paket */
      dsl->raw_data[dsl->data_counter++] = b;
      dsl->check_sum += b;
   }

   /* always remember last byte received for
      detection of sync condition */
   dsl->last_byte = b;

out:
   return status;
}

