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

 Thread Utils Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology
 Most of the code is taken from: http://linux.die.net/man/3/pthread_getschedparam

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <pthread.h>
#include <stdio.h>


static void display_sched_attr(int policy, struct sched_param *param)
{
    fprintf(stderr, "    policy=%s, priority=%d\n",
            (policy == SCHED_FIFO)  ? "SCHED_FIFO" :
            (policy == SCHED_RR)    ? "SCHED_RR" :
            (policy == SCHED_OTHER) ? "SCHED_OTHER" :
            "???",
            param->sched_priority);
}


void display_thread_sched_attr(const char *msg)
{
    int policy;
    struct sched_param param;
    pthread_getschedparam(pthread_self(), &policy, &param);
    fprintf(stderr, "%s\t", msg);
    display_sched_attr(policy, &param);
}

