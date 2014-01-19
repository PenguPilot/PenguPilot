/*__________________________________
 |       _         _         _      |
 |     _( )__    _( )__    _( )__   |
 |   _|     _| _|     _| _|     _|  |
 |  (_   S (_ (_   C (_ (_   L (_   |
 |    |_( )__|  |_( )__|  |_( )__|  |
 |                                  |
 | Signaling and Communication Link |
 |__________________________________|

 SCL C Binding Implementation

 Copyright (C) 2014 Tobias Simon

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <assert.h>
#include <unistd.h>
#include <stdio.h>

#include <yaml.h>
#include <glib.h>
#include <zmq.h>

#include "scl.h"


static GHashTable *params_ht = NULL;
static void *context = NULL;


void add_socket(char *name, char *path, int type)
{
   assert(name);
   assert(path);
   assert(type != -1);
   void *socket = zmq_socket(context, type);
   assert(socket);
   switch (type)
   {
      case ZMQ_REP:
      case ZMQ_PUB:
         zmq_bind(socket, path);
         break;

      case ZMQ_SUB:
         zmq_setsockopt(socket, ZMQ_SUBSCRIBE, "", 0);

      case ZMQ_REQ:
         zmq_connect(socket, path);
   }

   g_hash_table_insert(params_ht, name, socket);
}



int scl_init(char *comp_name)
{
   char *generator = getenv("SCL_CACHE_GENERATOR");
   if (generator == NULL)
   {
      fprintf(stderr, "environment variable SCL_CACHE_GENERATOR is undefined\n");
      return -1;
   }
   
   char cache[128];
   char *home = getenv("HOME");
   if (home == NULL)
   {
      fprintf(stderr, "environment variable HOME is undefined\n");
      return -2;
   }
   sprintf(cache, "%s/.SCL/scl.yaml", home);

   int result = system(generator);
   if (result != 0)
   {
      fprintf(stderr, "generator script failed\n");
      return -2;
   }

   context = zmq_init(1);
   params_ht = g_hash_table_new(g_str_hash, g_str_equal);
   FILE *source = fopen(cache, "rb");
    
   yaml_parser_t parser;
   yaml_parser_initialize(&parser);
   yaml_parser_set_input_file(&parser, source);
    
   yaml_document_t document;
   yaml_parser_load(&parser, &document);
    
   yaml_node_t *root = yaml_document_get_root_node(&document);
   assert(root->type == YAML_MAPPING_NODE);
   int found = 0;

   for (yaml_node_pair_t *i = root->data.mapping.pairs.start;
        i != root->data.mapping.pairs.top;
        i++)
   {
      yaml_node_t *node = yaml_document_get_node(&document, i->value);
      assert(node->type == YAML_SEQUENCE_NODE);
      char *name = (char *)yaml_document_get_node(&document, i->key)->data.scalar.value;
      if (strcmp(name, comp_name) == 0)
      {
         found = 1;
         for (yaml_node_item_t *j = node->data.sequence.items.start;
              j != node->data.sequence.items.top;
              j++)
         {
            yaml_node_t *list_node = yaml_document_get_node(&document, *j);
            assert(list_node->type == YAML_MAPPING_NODE);
            char *gate_name = NULL;
            char *ipc_path = NULL;
            int socket_type = -1;
            for (yaml_node_pair_t *k = list_node->data.mapping.pairs.start;
                 k != list_node->data.mapping.pairs.top;
                 k++)
            {
               char *key_name = (char *)yaml_document_get_node(&document, k->key)->data.scalar.value;
               char *val_name = (char *)yaml_document_get_node(&document, k->value)->data.scalar.value;
               if (strcmp(key_name, "gate_name") == 0)
               {
                  assert(gate_name == NULL);
                  gate_name = val_name;
               }
               else if (strcmp(key_name, "zmq_socket_path") == 0)
               {
                  assert(ipc_path == NULL);
                  ipc_path = val_name;
               }
               else
               {
                  assert(socket_type == -1);
                  assert(strcmp(key_name, "zmq_socket_type") == 0);
                  socket_type = atoi(val_name);
               }
            }
            add_socket(gate_name, ipc_path, socket_type);
         }
      }
   }
   if (!found)
   {
      fprintf(stderr, "could not find SCL cache: %s\n", cache);
      return -3;   
   }

   sleep(1); /* give scl some time to establish
                a link between publisher and subscriber */
   return 0;
}


void *scl_get_context(void)
{
   return context;
}


void *scl_get_socket(char *gate)
{
   return g_hash_table_lookup(params_ht, gate);
}


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
int scl_send_static(void *socket, void *data, size_t len)
{
    return _zmq_send(socket, data, len, 0, 0);
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

