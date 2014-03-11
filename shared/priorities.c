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
  
 Process Scheduler Configuration Library Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include <yaml.h>
#include <sched.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <assert.h>

#include <util.h>


#include "priorities.h"


int set_priority(char *name)
{
   /* construct config file path: */
   char buffer[1024];
   user_data_dir(buffer);
   strcat(buffer, "/priorities.yaml");

   /* open config file: */
   FILE *source = fopen(buffer, "rb");
   if (!source)
   {
      return -1;
   }
   
   /* parse config file: */
   yaml_parser_t parser;
   yaml_parser_initialize(&parser);
   yaml_parser_set_input_file(&parser, source);
   yaml_document_t document;
   if (yaml_parser_load(&parser, &document) != 1)
   {
      return -1;
   }

   /* traverse config file and retrieve priority: */
   yaml_node_t *root = yaml_document_get_root_node(&document);
   assert(root);
   assert(root->type == YAML_MAPPING_NODE);
   int prio = -1;
   for (yaml_node_pair_t *i = root->data.mapping.pairs.start;
        i != root->data.mapping.pairs.top;
        i++)
   {
      yaml_node_t *key = yaml_document_get_node(&document, i->key);
      assert(key->type == YAML_SCALAR_NODE);
      yaml_node_t *val = yaml_document_get_node(&document, i->value);
      assert(val->type == YAML_SCALAR_NODE);
      if (strcmp((char *)key->data.scalar.value, name) == 0)
      {
         prio = atoi((char *)val->data.scalar.value);
         break;
      }
   }

   /* free allocated memory and close files: */
   yaml_document_delete(&document);
   yaml_parser_delete(&parser);
   fclose(source);
   
   /* set priority */
   struct sched_param sp;
   assert(prio <= sched_get_priority_max(SCHED_FIFO));
   assert(prio >= sched_get_priority_min(SCHED_FIFO));
   sp.sched_priority = prio;
   
   int result = sched_setscheduler(getpid(), SCHED_FIFO, &sp);
   if (result < 0)
   {
      return result;
   }
   result = mlockall(MCL_CURRENT | MCL_FUTURE);
   return result;
}

