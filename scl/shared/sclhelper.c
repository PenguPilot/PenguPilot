/*__________________________________
 |       _         _         _      |
 |     _( )__    _( )__    _( )__   |
 |   _|     _| _|     _| _|     _|  |
 |  (_   S (_ (_   C (_ (_   L (_   |
 |    |_( )__|  |_( )__|  |_( )__|  |
 |                                  |
 | Signaling and Communication Link |
 |__________________________________|

 SCL Helper Functions

 Copyright (C) 2010 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <stddef.h>
#include <string.h>
#include <malloc.h>
#include <zmq.h>


#if ZMQ_VERSION_MAJOR <= 2
   #define zmq_sendmsg zmq_send
   #define zmq_recvmsg zmq_recv
#endif


static void _simple_free(void *data, void *hint);
static int _zmq_send(void *socket, void *data, size_t len, int free_str, int arg);

/*
 * send data to a zmq socket and
 * automatically free it when it was sent
 */
int scl_send_dynamic(void *socket, void *data, size_t len, int arg)
{
    return _zmq_send(socket, data, len, 1, arg);
}


/*
 * send data to a zmq socket and
 * automatically free it when it was sent
 */
int scl_copy_send_dynamic(void *socket, void *data, size_t len)
{
    char *buffer = malloc(len);
    memcpy(buffer, data, len);
    return _zmq_send(socket, buffer, len, 1, 0);
}


/*
 * send data to a zmq socket and
 * do not automatically free it when it was sent
 */
int scl_send_static(void *socket, void *data, size_t len, int arg)
{
    return _zmq_send(socket, data, len, 0, arg);
}


/*
 * receive data from zmq socket into a buffer
 */
int scl_recv_static(void *socket, void *buffer, size_t buf_size)
{
   zmq_msg_t message;
   zmq_msg_init(&message);
   int result = zmq_recvmsg(socket, &message, 0);
   if (result < 0)
   {
      goto out;
   }
   size_t size = zmq_msg_size(&message);
   if (size > buf_size)
   {
      result = -EINVAL;
      goto out;
   }
   result = (int)size;
   memcpy(buffer, zmq_msg_data(&message), size);

out:
   zmq_msg_close(&message);
   return result;
}


/*
 * receive data from a zmq buffer and return the data as dynamic memory
 * (has to be freed by caller)
 */
void *scl_recv_dynamic(void *socket, size_t *size_out)
{
   char *buffer = NULL;
   zmq_msg_t message;
   zmq_msg_init(&message);
   int result = zmq_recvmsg(socket, &message, 0);
   if (result < 0)
   {
      goto out;
   }
   size_t size = zmq_msg_size(&message);
   buffer = malloc(size);
   memcpy(buffer, zmq_msg_data(&message), size);
   *size_out = size;

out:
   zmq_msg_close(&message);
   return buffer;    
    
}

static void _simple_free(void *data, void *hint)
{
   (void)hint;
   free(data);
}


static int _zmq_send(void *socket, void *data, size_t len, int free_str, int arg)
{
   zmq_msg_t message;
   zmq_msg_init_data(&message, data, len, free_str ? _simple_free : NULL, NULL);
   int ret = zmq_sendmsg(socket, &message, arg);
   zmq_msg_close(&message);
   return ret;
}

