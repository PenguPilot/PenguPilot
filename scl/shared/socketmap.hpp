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


#ifndef __SOCKETMAP_HPP__
#define __SOCKETMAP_HPP__


#include <string>
#include <set>
#include <map>
#include <stdexcept>
#include <zmq.hpp>
#include <yaml-cpp/yaml.h>


using namespace std;


class SocketMapException: public runtime_error
{
public:
   SocketMapException(const string& msg) : runtime_error(msg)
   {
   }
};


class SocketMap
{
public:
   SocketMap(string component_name);
   static SocketMap &instance(string component)
   {
      static SocketMap _instance(component);
      return _instance;
   }

   void *get_context(void);
   zmq::socket_t *operator[](string gate);
   ~SocketMap();

private:
   
   SocketMap(SocketMap const&);
   void operator=(SocketMap const&);

   struct SocketDesc
   {
      string gate_name;
      string socket_path;
      int socket_type;
   };

   typedef std::map<std::string, zmq::socket_t *> MapType;
   MapType map;
   zmq::context_t *ctx;

   void copy_desc_from_node(const YAML::Node &node, SocketDesc &desc);
   void configure_sockets(std::string component);
   void configure_socket(SocketDesc &desc);
};


#endif /* __SOCKETMAP_HPP__ */

