/*__________________________________
 |       _         _         _      |
 |     _( )__    _( )__    _( )__   |
 |   _|     _| _|     _| _|     _|  |
 |  (_   S (_ (_   C (_ (_   L (_   |
 |    |_( )__|  |_( )__|  |_( )__|  |
 |                                  |
 | Signaling and Communication Link |
 |__________________________________|

 Socket Map Class

 Copyright (C) 2010 Tobias Simon, Andre Puschmann, Mikhail Tarasov,
                    Jin Yang, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib> 
#include <unistd.h>
#include <set>
#include <map> 
#include <cstring>

#include "socketmap.hpp"


using namespace std;


void *SocketMap::get_context(void)
{
   return (void *)(ctx); /* fix needed? */
}


zmq::socket_t *SocketMap::operator[](string gate)
{
   if(map.find(gate) != map.end())
   {
       return map[gate];
   }
   else
   {
       return NULL;
   }
}


void SocketMap::copy_desc_from_node(const YAML::Node &node, SocketDesc &desc)
{
   node["gate_name"] >> desc.gate_name;
   node["zmq_socket_path"] >> desc.socket_path;
   node["zmq_socket_type"] >> desc.socket_type;
}


SocketMap::SocketMap(string component)
{
   ctx = new zmq::context_t(1);
   configure_sockets(component);
}


SocketMap::~SocketMap()
{
   for (SocketMap::MapType::iterator itr = map.begin(); itr != map.end(); itr++)
   {
      delete itr->second;
   }
   delete ctx;
}


void SocketMap::configure_sockets(std::string component)
{
   char *generator = getenv("SCL_CACHE_GENERATOR");
   if (generator == NULL)
   {
      throw SocketMapException("environment variable SCL_CACHE_GENERATOR is undefined");
   }
   
   char cache[128];
   char *home = getenv("HOME");
   if (home == NULL)
   {
      throw SocketMapException("environment variable HOME is undefined");
   }
   sprintf(cache, "%s/.SCL/scl.yaml", home);

   int result = system(generator);
   if (result != 0)
   {
      throw SocketMapException("generator script failed");
   }

   std::ifstream in(cache);
   YAML::Parser parser(in);
   YAML::Node doc;
   result = parser.GetNextDocument(doc);
   if (result != 1)
   {
      throw SocketMapException("could not parse " + std::string(cache));
   }

   for (YAML::Iterator comp_it = doc.begin(); comp_it != doc.end(); comp_it++)
   {
      string current_component;
      comp_it.first() >> current_component;
      if (component == current_component)
      {
         const YAML::Node &gates = comp_it.second();
         for (YAML::Iterator gates_it = gates.begin(); gates_it != gates.end(); gates_it++)
         {
            SocketDesc desc;
            copy_desc_from_node(*gates_it, desc);
            configure_socket(desc);
         }
         return;
      }
   }
   throw SocketMapException(component + " not found in system configuration (system.yaml)");
}


void SocketMap::configure_socket(SocketDesc &desc)
{
   zmq::socket_t *zmq_sock = new zmq::socket_t(*ctx, desc.socket_type);
   const char *path = desc.socket_path.c_str();
   switch (desc.socket_type)
   {
      case ZMQ_REP:
      case ZMQ_PUB:
         zmq_sock->bind(path);
         break;

      case ZMQ_SUB:
         zmq_sock->setsockopt(ZMQ_SUBSCRIBE, "", 0);

      case ZMQ_REQ:
         zmq_sock->connect(path);
   }
   map[desc.gate_name] = zmq_sock;
}



extern "C"
{
    SocketMap *socket_map = NULL;

    int scl_init(char *component)
    {
       if (socket_map == NULL)
       {
          try
          {
             socket_map = &SocketMap::instance(component);
          }
          catch (SocketMapException &e)
          {
             cerr << "error in socket map: " << e.what() << endl;
             socket_map = NULL;
             return -1;
          }
          catch (exception &e)
          {
             cerr << "unknown exception: " << e.what() << endl;
             socket_map = NULL;
             return -2;
          }
       }
       return 0;
    }


    void scl_close(void *socket)
    {
        zmq_close(socket);
    }


    void *scl_get_context(void)
    {
       if (socket_map == NULL)
       {
          return NULL;
       }
       return socket_map->get_context();
    }


    void scl_term(void)
    {
        zmq_term(scl_get_context());
    }


    void *scl_get_socket(char *gate)
    {
       if (socket_map == NULL)
       {
          return NULL;
       }
       zmq::socket_t *socket = (*socket_map)[string(gate)];
       if (socket == NULL)
       {
          return NULL;
       }
       return (void *)(*socket);
    }
}
