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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <net/if.h>


#include "udp4.h"


udp_socket_t *udp_socket_create(char *udp_host, int udp_port, int ttl, int do_bind)
{
   (void)ttl;
   udp_socket_t *udp_socket;
   int one = 1;

   /* malloc() some memory */
   if(!(udp_socket = (udp_socket_t*)malloc(sizeof(udp_socket_t))))
   {
      perror("malloc");
      exit(EXIT_FAILURE);
   }
   memset(udp_socket, 0, sizeof(*udp_socket));

   /* create sin_addr, beware of net/host byte-order */
   udp_socket->sin.sin_family = AF_INET;
   udp_socket->sin.sin_port = htons(udp_port);

   if(!inet_pton(AF_INET, udp_host, &(udp_socket->sin.sin_addr.s_addr)))
   {
      perror("inet_pton");
      exit(EXIT_FAILURE);
   }

   /* creating socket */
   if((udp_socket->sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
   {
      perror("socket");
      exit(EXIT_FAILURE);
   }

   /* set SO_REUSEADDR */
   if(setsockopt(udp_socket->sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)))
      perror("setsockopt (SO_REUSEADDR)");

   /* set SO_BROADCAST */
   if(setsockopt(udp_socket->sock, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one)))
      perror("setsockopt (SO_BROADCAST)");


   /* binding socket to network */
   if(do_bind)
   {
      if(bind(udp_socket->sock, (struct sockaddr *)&udp_socket->sin, sizeof(udp_socket->sin)))
      {
         perror("bind");
         exit(EXIT_FAILURE);
      }
   }

   return udp_socket;
}

int udp_socket_send(udp_socket_t *udp_socket, void *data, unsigned int len)
{
   int result = (int)sendto(udp_socket->sock, data, len, 0, (struct sockaddr *)&udp_socket->sin, sizeof(udp_socket->sin));
   return result;
}

int udp_socket_recv(udp_socket_t *udp_socket, void *data, unsigned int len, struct sockaddr_in *from)
{
   socklen_t sockaddr_size = sizeof(*from);
   fd_set readfds;
   struct timeval timeout, *timeout_ptr = NULL;
   int packet_len, retval;

   /* timeout set? */
   if(udp_socket->timeout)
   {
      timeout.tv_sec = udp_socket->timeout;
      timeout.tv_usec = 0;
      timeout_ptr = &timeout;
   }

   /* poll socket till there is some input */
   FD_ZERO(&readfds);
   FD_SET(udp_socket->sock, &readfds);
   retval = select(FD_SETSIZE, &readfds, NULL, NULL, timeout_ptr);

   if(retval == -1)
   {
      perror("select");
      return retval;
   }
   else if(retval == 0)
   {
      fprintf(stderr, "Timeout hit while waiting for data\n");
      return retval;
   }

   /* got one! (packet) */
   packet_len = (int)recvfrom(udp_socket->sock, data, len, 0, (struct sockaddr *)from, &sockaddr_size);
   if(packet_len <= 0)
   {
      perror("recvfrom");
   }

   return packet_len;
}

void udp_socket_close(udp_socket_t *udp_socket)
{
   close(udp_socket->sock);
   free(udp_socket);
}
