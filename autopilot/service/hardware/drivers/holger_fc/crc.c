/*
 * crc.c
 *
 * http://mikrokopter.de/ucwiki/en/SerialProtocol#head-58f4dd9dd018e8c7f237035c08aad0269ea05f12
 *
 *  Created on: 14.06.2010
 *      Author: tobi
 */


#include "crc.h"


crc_t calc_crc(const char *data, const unsigned int len)
{
   unsigned int i;
   crc_t crc = 0;
   for (i = 0; i < len; i++)
   {
      crc = crc + data[i];
   }
   crc %= 4096;
   return crc;
}


int crc_ok(const crc_t crc, const char *crc_chars)
{
   char hcrc = '=' + crc / 64;
   char lcrc = '=' + crc % 64;
   return    hcrc == crc_chars[0]
             && lcrc == crc_chars[1];
}


int calc_crc_chars(char *crc_chars, const crc_t crc)
{
   crc_chars[0] = '=' + crc / 64;
   crc_chars[1] = '=' + crc % 64;
   return 2;
}

