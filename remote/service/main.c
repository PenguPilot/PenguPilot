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
  
 Remote Control Service Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <unistd.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <msgpack.h>

#include <daemon.h>
#include <util.h>
#include <scl.h>
#include <opcd_interface.h>
#include <serial.h>

#include "rc_dsl.h"
#include "sbus_parser.h"
#include "sbus_serial.h"


#define MAX_CHANNELS 16


static int running = 1;
static msgpack_sbuffer *msgpack_buf = NULL;
static msgpack_packer *pk = NULL;
static char *platform = NULL;
static void *rc_socket = NULL;


int dsl_run(void)
{  
   THROW_BEGIN();

   char buffer_path[128];
   char buffer_speed[128];
   strcpy(buffer_path, platform);
   strcpy(buffer_speed, platform);
   strcat(buffer_path, ".dsl_serial.path");
   strcat(buffer_speed, ".dsl_serial.speed");
   char *dev_path;
   opcd_param_get(buffer_path, &dev_path);
   int dev_speed;
   opcd_param_get(buffer_speed, &dev_speed);
   serialport_t port;
   THROW_ON_ERR(serial_open(&port, dev_path, dev_speed, O_RDONLY));
   
   /* init parser: */
   rc_dsl_t rc_dsl;
   rc_dsl_init(&rc_dsl);

   while (running)
   {
      int b = serial_read_char(&port);
      if (b < 0)
      {
         msleep(1);
         continue;
      }
      int status = rc_dsl_parse_dsl_data(&rc_dsl, (uint8_t)b);
      if (status == 1)
      {
         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, MAX_CHANNELS + 1);
         PACKI(RC_DSL_RSSI_VALID(rc_dsl.RSSI));    /* index 0: valid */
         PACKFV(rc_dsl.channels, MAX_CHANNELS); /* index 1, .. : channels */
         scl_copy_send_dynamic(rc_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
 
   THROW_END();
}


int sbus_run(void)
{  
   THROW_BEGIN();
   char buffer_path[128];
   strcpy(buffer_path, platform);
   strcat(buffer_path, ".sbus_serial.path");
   char *dev_path;
   opcd_param_get(buffer_path, &dev_path);
   int fd = sbus_serial_open(dev_path);
   THROW_IF(fd < 0, -ENODEV);
   
   /* init parser: */
   sbus_parser_t parser;
   sbus_parser_init(&parser);

   while (running)
   {
      int b = sbus_serial_read(fd);
      if (b < 0)
      {
         msleep(1);
         continue;
      }
      bool status = sbus_parser_process(&parser, b);
      if (status)
      {
         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, MAX_CHANNELS + 1);
         PACKI((int)parser.valid);    /* index 0: valid */
         PACKFV(parser.channels, MAX_CHANNELS); /* index 1, .. : channels */
         scl_copy_send_dynamic(rc_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
 
   THROW_END();
}


int _main(void)
{
   THROW_BEGIN();

   /* initialize msgpack buffers: */
   msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);
   pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   THROW_IF(pk == NULL, -ENOMEM);
  
   /* initialize SCL: */
   THROW_ON_ERR(scl_init("remote"));
   rc_socket = scl_get_socket("remote");
   THROW_IF(rc_socket == NULL, -EIO);

   /* init opcd and get plaform string: */
   opcd_params_init("", 0);
   opcd_param_get("platform", &platform);
   
   /* determine the receiver type: */
   char type_buf[128];
   strcpy(type_buf, platform);
   strcat(type_buf, ".rx_type");
   char *type = NULL;
   opcd_param_get(type_buf, &type);
   THROW_IF(type == NULL, -EINVAL);
   
   /* start the receiver code: */
   if (strcmp(type, "dsl") == 0)
   {
      THROW_ON_ERR(dsl_run());
   }
   else if (strcmp(type, "sbus") == 0)
   {
      THROW_ON_ERR(sbus_run());
   }
   
   THROW_END();
}


void _cleanup(void)
{
   running = 0;
}


void main_wrap(int argc, char *argv[])
{
   (void)argc;
   (void)argv;

   exit(-_main());   
}


int main(int argc, char *argv[])
{
   char pid_file[1024];
   sprintf(pid_file, "%s/.PenguPilot/run/remote.pid", getenv("HOME"));
   daemonize(pid_file, main_wrap, _cleanup, argc, argv);
   return 0;
}

