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
  
 G-Vector Filter Array Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <math.h>
#include <stdio.h>
#include <string.h>

#include <util.h>
#include <pthread.h>
#include <physics.h>
#include <scl.h>
#include <msgpack.h>

#include "../util/math/conv.h"
#include "../util/logger/logger.h"

#include "gvfa.h"


static float gvfa[36][72][72][3];
static float prev_state[3] = {0.0f, 0.0f, G_CONSTANT};
static void *gvfa_pub_socket = NULL;
static msgpack_sbuffer *msgpack_buf = NULL;
static msgpack_packer *pk = NULL;
static int prev_i = -1;
static int prev_j = -1;
static int prev_k = -1;


static void _gvfa_init(void)
{
   FOR_N(i, 36)
      FOR_N(j, 72)
         FOR_N(k, 72)
         {
            gvfa[i][j][k][0] = 0.0;
            gvfa[i][j][k][1] = 0.0;
            gvfa[i][j][k][2] = G_CONSTANT;
         }
}


void *gvfa_save(void *arg)
{
   (void)arg;
   char path[1024];
   sprintf(path, "%s/.PenguPilot/config/gvfa_state.txt", getenv("HOME"));
   FILE *file = fopen(path, "w");
   int c = 0;
   FOR_N(i, 36)
      FOR_N(j, 72)
         FOR_N(k, 72)
         {
            if (gvfa[i][j][k][0] != 0.0f || gvfa[i][j][k][1] != 0.0f)
               c += 3;
            fprintf(file, "%d %d %d %f %f %f\n", i, j, k,
                    gvfa[i][j][k][0], gvfa[i][j][k][1], gvfa[i][j][k][2]);
         }
   LOG(LL_INFO, "saved %d filter states; %.1f%% coverage", c, 100.0f * (float)c * (float)sizeof(float) / (float)sizeof(gvfa));
   return NULL;
}


void gvfa_init(void)
{
   _gvfa_init();
   char path[1024];
   sprintf(path, "%s/.PenguPilot/config/gvfa_state.txt", getenv("HOME"));
   FILE *file = fopen(path, "r");
   if (file)
   {
      int c = 0;
      while (!feof(file))
      {
         int i, j, k;
         float x, y, z;
         char buffer[1024];
         if (fgets(buffer, 1024, file))
         {
            if (sscanf(buffer, "%d %d %d %f %f %f", &i, &j, &k, &x, &y, &z) == 6)
            {
               if (x != 0.0f || y != 0.0f)
                  c += 3;
               gvfa[i][j][k][0] = x;
               gvfa[i][j][k][1] = y;
               gvfa[i][j][k][2] = z;
            }
         }
      }
      fclose(file);
      LOG(LL_INFO, "loaded %d filter states; %.1f%% coverage", c, 100.0f * (float)c * (float)sizeof(float) / (float)sizeof(gvfa));
   }

   /* open gvfa_pubitoring socket: */
   gvfa_pub_socket = scl_get_socket("gvfa");
   ASSERT_NOT_NULL(gvfa_pub_socket);
   int64_t hwm = 1;
   zmq_setsockopt(gvfa_pub_socket, ZMQ_SNDHWM, &hwm, sizeof(hwm));

   /* init msgpack buffer: */
   ASSERT_NULL(msgpack_buf);
   msgpack_buf = msgpack_sbuffer_new();
   ASSERT_NOT_NULL(msgpack_buf);
   ASSERT_NULL(pk);
   pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
}


void gvfa_calc(float out[3], float in[3], float p, float r, float y)
{
   int i = (int)((rad2deg(p) + 90.0) / 5.0);
   int j = (int)((rad2deg(r) + 180.0) / 5.0);
   int k = (int)(rad2deg(y) / 5.0);
   float _x = gvfa[i][j][k][0];
   float _y = gvfa[i][j][k][1];
   if (_x == 0.0f && _y == 0.0f)
   {
      FOR_N(l, 3)
         gvfa[i][j][k][l] = prev_state[l];
      static int x = 0;
   }
  
   if ((i != prev_i || j != prev_j || k != prev_k) &&
       prev_i != -1 && prev_j != -1 && prev_k != -1)
   {
      msgpack_sbuffer_clear(msgpack_buf);
      msgpack_pack_array(pk, 6);
      PACKF(prev_state[0]); /* 0 */
      PACKF(prev_state[1]); /* 1 */
      PACKF(prev_state[2]); /* 2 */
      PACKI(prev_i); /* 3 */
      PACKI(prev_j); /* 4 */
      PACKI(prev_k); /* 5 */
      scl_copy_send_dynamic(gvfa_pub_socket, msgpack_buf->data, msgpack_buf->size);
   }

   FOR_N(l, 3)
   {
      float a = 0.01;
      float *state = &gvfa[i][j][k][l];
      *state = *state * (1.0f - a) + in[l] * a;
      prev_state[l] = *state;
      out[l] = in[l] - *state;
   }
 

   prev_i = i;
   prev_j = j;
   prev_k = k;
}

