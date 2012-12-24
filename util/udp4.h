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
  
 UDP Library

 Copyright (C) 2010 Jan Roemisch, Ilmenau University of Technology
 Copyright (C) 2010 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


/*
 * convenience record for combination of socket information
 */
typedef struct udp_socket_s
{
   int sock;
   struct sockaddr_in sin;
   int timeout;
}
udp_socket_t;


udp_socket_t *udp_socket_create(char *udp_host, int udp_port, int udp_ttl, int bind);

int udp_socket_send(udp_socket_t *udp_socket, void *data, unsigned int len);

int udp_socket_recv(udp_socket_t *udp_socket, void *data, unsigned int len, struct sockaddr_in *from);

void udp_socket_close(udp_socket_t *udp_socket);

