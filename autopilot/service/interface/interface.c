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
  
 Autopilot Command Interface

 Copyright (C) 2011 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <stdio.h>
#include <malloc.h>
#include <math.h>
#include <unistd.h>

#include <simple_thread.h>
#include <pilot.pb-c.h>
#include <util.h>
#include <scl.h>

#include "interface.h"
#include "../platform/platform.h"
#include "../util/logger/logger.h"
#include "../control/position/u_ctrl.h"
#include "../control/position/navi.h"
#include "../control/position/yaw_ctrl.h"
#include "../main_loop/main_loop.h"

#define THREAD_NAME     "interface"
#define THREAD_PRIORITY 97


static gps_data_t gps_start;
static void *cmd_socket = NULL;
static simple_thread_t thread;
static bool locked = false;


void gps_start_set(gps_data_t *gps_data)
{
   memcpy(&gps_start, gps_data, sizeof(gps_data_t));
}


void interface_lock(bool value)
{
   locked = value;
}




int set_ctrl_param(CtrlParam param, float value)
{
   int status = 0;
   switch (param)
   {
      case CTRL_PARAM__POS_E:
      {
         LOG(LL_DEBUG, "e pos update: %f", value);
         navi_set_dest_e(value); // can't fail
         break;
      }

      case CTRL_PARAM__POS_N:
      {
         LOG(LL_DEBUG, "n pos update: %f", value);
         navi_set_dest_n(value); // can't fail
         break;
      }

      case CTRL_PARAM__POS_U_GROUND:
      {
         LOG(LL_DEBUG, "u ground pos update: %f", value);
         //u_setpoint_t setpoint = {value, 1};
         //u_ctrl_set_setpoint(setpoint); // can't fail
         break;
      }

      case CTRL_PARAM__POS_U:
      {
         LOG(LL_DEBUG, "u pos update: %f", value);
         //u_setpoint_t setpoint = {value, 0};
         //u_ctrl_set_setpoint(setpoint);
         break;
      }

      case CTRL_PARAM__POS_YAW:
      {
         LOG(LL_DEBUG, "yaw pos update: %f", value);
         //status = yaw_ctrl_set_pos(value);
         break;
      }
      
      case CTRL_PARAM__SPEED_NE:
      {
         LOG(LL_DEBUG, "ne speed update: %f", value);
         status = navi_set_travel_speed(value);
         break;
      }

      case CTRL_PARAM__SPEED_U:
      {
         LOG(LL_ERROR, "[not implemented] z speed update: %f", value);
         break;
      }

      case CTRL_PARAM__SPEED_YAW:
      {
         LOG(LL_DEBUG, "[not implemented] yaw speed update: %f", value);
         break;
      }
   }
   return status;
}



static void check_and_set_ctrl_param(PilotRep *reply, PilotReq *request)
{
   if (request->ctrl_data == NULL)
   {
      reply->err_msg = "no ctrl_data provided";
      LOG(LL_ERROR, reply->err_msg);
      reply->status = STATUS__E_SEMANTIC;
   }
   else if (locked)
   {
      reply->err_msg = "interface locked";   
      LOG(LL_ERROR, reply->err_msg);
      reply->status = STATUS__E_SEMANTIC;
   }
   else if (set_ctrl_param(request->ctrl_data->param, request->ctrl_data->val) != 0)
   {
      reply->err_msg = "parameter value out of range";
      LOG(LL_ERROR, reply->err_msg);
      reply->status = STATUS__E_SEMANTIC;
   }
   else
   {
      reply->status = STATUS__OK;
   }
}


static void get_state(Params *params)
{
   params->start_lon = gps_start.lon;
   params->start_lat = gps_start.lat;
   params->start_alt = gps_start.alt;
   /*
   params->setp_x = navi_get_dest_e();
   params->setp_y = navi_get_dest_n();
   float setp_u = u_ctrl_get_setpoint();
   if (u_ctrl_mode_is_ground())
   {
      params->setp_u_ground = setp_u;
   }
   else
   {
      params->setp_u = setp_u;
   }
   params->setp_yaw = 0.0;
   */
}


SIMPLE_THREAD_BEGIN(thread_func)
{
   SIMPLE_THREAD_LOOP_BEGIN
   {
      PilotRep reply = PILOT_REP__INIT;
      reply.status = STATUS__OK;
      unsigned char raw_data[1024];
      int raw_data_size = scl_recv_static(cmd_socket, raw_data, sizeof(raw_data));
      if (raw_data_size < 0)
      {
         LOG(LL_ERROR, "scl recv failed");
         sleep(1);
         continue;
      }
      PilotReq *request = pilot_req__unpack(NULL, raw_data_size, raw_data);
      Params params = PARAMS__INIT;
      if (request == NULL)
      {
         reply.status = STATUS__E_SYNTAX;
         reply.err_msg = "could not parse protobuf message";
         LOG(LL_ERROR, reply.err_msg);
      }
      else
      {
         switch (request->type)
         {
            case REQUEST_TYPE__RESET_CTRL:
               LOG(LL_DEBUG, "RESET_CTRL");
               //ctrl_reset();
               break;

            case REQUEST_TYPE__SET_CTRL_PARAM:
               LOG(LL_DEBUG, "SET_CTRL_PARAM");
               check_and_set_ctrl_param(&reply, request);
               break;

            case REQUEST_TYPE__GET_PARAMS:
               LOG(LL_DEBUG, "GET_PARAMS");
               reply.params = &params;
               get_state(&params);
               break;

            default:
               LOG(LL_DEBUG, "unknown request type");
         }
         pilot_req__free_unpacked(request, NULL);
      }
      SCL_PACK_AND_SEND_DYNAMIC(cmd_socket, pilot_rep, reply);
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END


int cmd_init(void)
{
   ASSERT_ONCE();
   cmd_socket = scl_get_socket("ctrl");
   if (cmd_socket == NULL)
   {
      return -1;
   }
   simple_thread_start(&thread, thread_func, THREAD_NAME, THREAD_PRIORITY, NULL);
   return 0;
}

