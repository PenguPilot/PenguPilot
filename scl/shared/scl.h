/*__________________________________
 |       _         _         _      |
 |     _( )__    _( )__    _( )__   |
 |   _|     _| _|     _| _|     _|  |
 |  (_   S (_ (_   C (_ (_   L (_   |
 |    |_( )__|  |_( )__|  |_( )__|  |
 |                                  |
 | Signaling and Communication Link |
 |__________________________________|
 
 SCL C Binding Interface

 Copyright (C) 2014 Tobias Simon

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __SCL_H__
#define __SCL_H__


#include <malloc.h>
#include <zmq.h>


int scl_init(char *component);

void *scl_get_context(void);

void *scl_get_socket(char *gate);


/*
 * send data to a zmq socket and
 * automatically free it when it was sent
 */
int scl_send_dynamic(void *socket, void *data, size_t len, int arg);

int scl_copy_send_dynamic(void *socket, void *data, size_t len);


#define SCL_PACK_AND_SEND_DYNAMIC(socket, type, var) \
do { \
   unsigned int reply_data_len = type##__get_packed_size(&var); \
   void *buffer = malloc(reply_data_len); \
   type##__pack(&var, buffer); \
   scl_send_dynamic(socket, buffer, reply_data_len, 0); \
} while (0)


#define PACKB(val) msgpack_pack_bool(pk, val) /* pack bool */
#define PACKI(val) msgpack_pack_int(pk, val) /* pack integer */
#define PACKF(val) msgpack_pack_float(pk, val) /* pack float */
#define PACKD(val) msgpack_pack_double(pk, val) /* pack double */
#define PACKFV(ptr, n) FOR_N(i, n) PACKF(ptr[i]) /* pack float vector */


/*
 * send data to a zmq socket and
 * do not automatically free it when it was sent
 */
int scl_send_static(void *socket, void *data, size_t len);


/*
 * receive data from zmq socket into a buffer
 */
int scl_recv_static(void *socket, void *buffer, size_t buf_size);


/*
 * receive data from a zmq buffer and return the data as dynamic memory
 * (has to be freed by caller)
 */
void *scl_recv_dynamic(void *socket, size_t *size_out);


#define SCL_RECV_AND_UNPACK_DYNAMIC(var, socket, type) \
do { \
   unsigned char raw_data[1024]; \
   int raw_data_size = scl_recv_static(socket, raw_data, sizeof(raw_data)); \
   if (raw_data_size > 0) \
   { \
      var = type##__unpack(NULL, raw_data_size, raw_data); \
   } \
   else \
   { \
      var = NULL; \
   } \
} while (0)


#define SCL_FREE(type, var) \
do { \
   type##__free_unpacked(var, NULL); \
} \
while (0)


/* set protocol buffers optional attribute (see protoc-c output for details) */
#define PB_SET(var, attr, val) \
   var.has_##attr = 1;  \
   var.attr = val

#if ZMQ_VERSION_MAJOR < 3
   #define ZMQ_SNDHWM ZMQ_HWM
   #define ZMQ_RCVHWM ZMQ_HWM
#endif


#endif /* __SCL_H__ */
