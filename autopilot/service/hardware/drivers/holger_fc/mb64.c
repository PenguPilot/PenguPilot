/*
 * mb64.c
 *
 *  Created on: 14.06.2010
 *      Author: tobi
 */


#include "mb64.h"


int mb64_encode(char *output, const plchar_t *input, const size_t input_len)
{
   unsigned int in_len = input_len;
   unsigned int pt = 0;
   unsigned char a, b, c;
   unsigned char ptr = 0;
   while (in_len)
   {
      if (in_len)
      {
         a = input[ptr++];
         in_len--;
      }
      else
      {
         a = 0;
      }
      if (in_len)
      {
         b = input[ptr++];
         in_len--;
      }
      else
      {
         b = 0;
      }
      if (in_len)
      {
         c = input[ptr++];
         in_len--;
      }
      else
      {
         c = 0;
      }
      output[pt++] = '=' + (a >> 2);
      output[pt++] = '=' + (((a & 0x03) << 4) | ((b & 0xf0) >> 4));
      output[pt++] = '=' + (((b & 0x0f) << 2) | ((c & 0xc0) >> 6));
      output[pt++] = '=' + (c & 0x3f);
   }
   return pt; /* length of output */
}


int mb64_decode(plchar_t *output, const size_t len, const char *input)
{
   unsigned int in_len = len;
   unsigned char a, b, c, d;
   unsigned int out_pos = 0;
   unsigned int in_pos = 0;
   unsigned char x, y, z;

   while (len)
   {
      a = input[in_pos++] - '=';
      b = input[in_pos++] - '=';
      c = input[in_pos++] - '=';
      d = input[in_pos++] - '=';

      x = (a << 2) | (b >> 4);
      y = ((b & 0x0f) << 4) | (c >> 2);
      z = ((c & 0x03) << 6) | d;
      if (in_len--)
      {
         output[out_pos++] = x;
      }
      else
      {
         break;
      }
      if (in_len--)
      {
         output[out_pos++] = y;
      }
      else
      {
         break;
      }
      if (in_len--)
      {
         output[out_pos++] = z;
      }
      else
      {
         break;
      }
   }
   return out_pos;
}

