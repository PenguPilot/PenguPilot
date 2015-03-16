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

 Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

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
#include <logger.h>

#include "interface.h"
#include "../flight_logic/auto_logic.h"
#include "../control/control.h"
#include "../main_loop/main_loop.h"

#define THREAD_NAME     "interface"
#define THREAD_PRIORITY 96


static gps_data_t gps_start;
static void *cmd_socket = NULL;
static simple_thread_t thread;
static bool locked = false;


void gps_start_set(const gps_data_t *gps_data)
{
   memcpy(&gps_start, gps_data, sizeof(gps_data_t));
}


void interface_lock(bool val)
{
   locked = val;
}


int set_ctrl_param(CtrlParam param, float val)
{
   int status = 0;
   switch (param)
   {
      case CTRL_PARAM__POS_N:
      {
         LOG(LL_DEBUG, "n pos update: %f", val);
         auto_logic_set_n(val);
         break;
      }

      case CTRL_PARAM__POS_E:
      {
         LOG(LL_DEBUG, "e pos update: %f", val);
         auto_logic_set_e(val);
         break;
      }

      case CTRL_PARAM__POS_U_GROUND:
      {
         LOG(LL_DEBUG, "u ground pos update: %f", val);
         auto_logic_set_u_ground(val);
         break;
      }

      case CTRL_PARAM__POS_U:
      {
         LOG(LL_DEBUG, "u msl pos update: %f", val);
         auto_logic_set_u_msl(val);
         break;
      }

      case CTRL_PARAM__POS_YAW:
      {
         LOG(LL_DEBUG, "yaw pos update: %f", val);
         auto_logic_set_yaw(val);
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
      reply->err_msg = "parameter val out of range";
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
            case REQUEST_TYPE__START_MOTORS:
               LOG(LL_DEBUG, "START_MOTORS");
               auto_logic_enable_motors(true);
               break;

            case REQUEST_TYPE__STOP_MOTORS:
               LOG(LL_DEBUG, "STOP_MOTORS");
               auto_logic_enable_motors(false);
               break;

            case REQUEST_TYPE__SET_CTRL_PARAM:
               LOG(LL_DEBUG, "SET_CTRL_PARAM");
               check_and_set_ctrl_param(&reply, request);
               break;

            case REQUEST_TYPE__RESET_CTRL:
               LOG(LL_DEBUG, "RESET_CTRL");
               highlevel_control_reset();
               break;
            
            case REQUEST_TYPE__GET_PARAMS:
               LOG(LL_DEBUG, "GET_PARAMS");
               {
                  Params *params = malloc(sizeof(Params));
                  params__init(params);
                  get_state(params);
                  reply.params = params;
               }
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
   cmd_socket = scl_get_socket("ap_ctrl", "rep");
   if (cmd_socket == NULL)
   {
      return -1;
   }
   simple_thread_start(&thread, thread_func, THREAD_NAME, THREAD_PRIORITY, NULL);
   return 0;
}

